"""
RTK to SfM Trajectory Alignment

Aligns Structure from Motion (SfM) trajectory to RTK ground truth using
Horn's method with Umeyama scale estimation. Optionally refines with ICP.

Pipeline:
  1. Load both CSVs (from sfm_to_csv.py and rtk_gnss_to_csv.py)
  2. Normalize both trajectories by first pose (first pose becomes origin)
  3. Subsample to equal length (temporal correspondence by index)
  4. Horn's method with scale: closed-form solution for s, R, t
  5. (Optional) ICP refinement on the already-scaled trajectory
  6. Compute alignment metrics and visualize

Output:
  - Aligned SfM trajectory CSV
  - Alignment report (scale, rotation, translation, RMSE)
  - 3D visualization (before/after)
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from typing import Optional
import sys
import argparse

# Local import -- horn_method.py must be in the same directory or on PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent))
from horn_method import horn_method_with_scale, compute_path_length, subsample


def load_trajectory(csv_path: str) -> pd.DataFrame:
    """Load trajectory CSV (works for both SfM and RTK formats).

    Args:
        csv_path: Path to trajectory CSV file.

    Returns:
        DataFrame with at least columns: x, y, z, r11..r33.
    """
    df = pd.read_csv(csv_path)
    required = {'x', 'y', 'z', 'r11', 'r12', 'r13', 'r21', 'r22', 'r23', 'r31', 'r32', 'r33'}
    missing = required - set(df.columns)
    assert not missing, f"CSV missing columns: {missing}"
    print(f"Loaded {csv_path}: {len(df)} poses")
    return df


def extract_positions_and_rotations(df: pd.DataFrame) -> tuple:
    """Extract positions (N,3) and rotation matrices (N,3,3) from dataframe.

    Args:
        df: Trajectory dataframe with columns x, y, z, r11-r33.

    Returns:
        (positions, rotations): arrays of shape (N,3) and (N,3,3).
    """
    positions = df[['x', 'y', 'z']].values.astype(np.float64)
    rot_cols = ['r11', 'r12', 'r13', 'r21', 'r22', 'r23', 'r31', 'r32', 'r33']
    rotations = df[rot_cols].values.astype(np.float64).reshape(-1, 3, 3)
    return positions, rotations


def normalize_by_first_pose(positions: np.ndarray, rotations: np.ndarray) -> tuple:
    """Make the first pose the origin: multiply every pose by inv(first_pose).

    For SE(3) pose (R, t), the inverse is (R^T, -R^T @ t).
    Applying to each pose: R_norm = R @ R0^T,  t_norm = (t - t0) @ R0^{-T} ... no:
    more precisely, p_norm_i = R0^T @ (p_i - p0), R_norm_i = R0^T @ R_i.

    Args:
        positions: (N, 3) positions.
        rotations: (N, 3, 3) rotation matrices.

    Returns:
        (positions_norm, rotations_norm): Normalized arrays, same shapes.
    """
    p0 = positions[0].copy()
    R0 = rotations[0].copy()
    R0_inv = R0.T  # orthogonal matrix inverse

    positions_norm = (positions - p0) @ R0_inv.T
    rotations_norm = np.array([R0_inv @ R for R in rotations])

    return positions_norm, rotations_norm


def remove_duplicates(df: pd.DataFrame) -> pd.DataFrame:
    """Remove consecutive duplicate positions (e.g. RTK static phase).

    Args:
        df: Trajectory dataframe.

    Returns:
        Deduplicated dataframe.
    """
    n_before = len(df)
    df_clean = df.drop_duplicates(subset=['x', 'y', 'z'], keep='first').reset_index(drop=True)
    n_removed = n_before - len(df_clean)
    if n_removed > 0:
        print(f"  Removed {n_removed} duplicate positions ({len(df_clean)} remaining)")
    return df_clean


def downsample_to_match(positions: np.ndarray, target_count: int) -> tuple:
    """Uniformly subsample positions to target_count points.

    Args:
        positions: (N, 3) array.
        target_count: Desired number of points.

    Returns:
        (downsampled_positions, indices): The subsampled array and original indices.
    """
    n = len(positions)
    if n <= target_count:
        return positions, np.arange(n, dtype=int)

    indices = np.round(np.linspace(0, n - 1, target_count)).astype(int)
    return positions[indices], indices


def align_trajectories(sfm_pos: np.ndarray, rtk_pos: np.ndarray) -> dict:
    """Align SfM trajectory to RTK using Horn's method with scale.

    Both inputs should already be normalized by their first pose.
    They must have the same length (subsample beforehand).

    The transformation is:  p_aligned = scale * R @ p_sfm + t

    Args:
        sfm_pos: (N, 3) normalized SfM positions (source, to be transformed).
        rtk_pos: (N, 3) normalized RTK positions (target, reference).

    Returns:
        dict with keys: rotation, translation, scale, rmse, errors.
    """
    assert len(sfm_pos) == len(rtk_pos), (
        f"Trajectories must have same length, got {len(sfm_pos)} and {len(rtk_pos)}"
    )

    # Diagnostic: compare path lengths before alignment
    sfm_path_len = compute_path_length(sfm_pos)
    rtk_path_len = compute_path_length(rtk_pos)
    if sfm_path_len > 0:
        print(f"  Path lengths: SfM={sfm_path_len:.4f}, RTK={rtk_path_len:.4f}")
        print(f"  Path length ratio (RTK/SfM): {rtk_path_len / sfm_path_len:.6f}")

    # Horn's method with Umeyama scale estimation
    R, t, scale = horn_method_with_scale(sfm_pos, rtk_pos)

    # Apply transformation
    sfm_aligned = scale * (sfm_pos @ R.T) + t

    # Per-point errors (corresponding by index -- temporal correspondence)
    errors = np.linalg.norm(sfm_aligned - rtk_pos, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))

    print(f"  Scale factor: {scale:.6f}")
    print(f"  RMSE: {rmse:.6f}")

    return {
        'rotation': R,
        'translation': t,
        'scale': scale,
        'rmse': rmse,
        'errors': errors,
        'aligned_positions': sfm_aligned,
    }


def try_icp_refinement(sfm_scaled: np.ndarray, rtk_pos: np.ndarray,
                       max_distance: float = 1.0,
                       max_iterations: int = 200) -> Optional[dict]:
    """Optionally refine alignment with ICP (requires open3d).

    Only applies a rigid transformation (no additional scale).
    The input sfm_scaled should already be at the correct scale.

    Args:
        sfm_scaled: (N, 3) SfM positions after scaling + Horn alignment.
        rtk_pos: (N, 3) RTK positions (target).
        max_distance: Max correspondence distance for ICP.
        max_iterations: Max ICP iterations.

    Returns:
        dict with refined rotation, translation, rmse, aligned_positions.
        Returns None if open3d is not available.
    """
    try:
        import open3d as o3d
    except ImportError:
        print("  open3d not installed, skipping ICP refinement")
        return None

    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(sfm_scaled)
    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(rtk_pos)

    result = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd,
        max_correspondence_distance=max_distance,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
            with_scaling=False  # scale already estimated by Horn
        ),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=max_iterations
        )
    )

    T = result.transformation
    R_icp = T[:3, :3]
    t_icp = T[:3, 3]

    refined = sfm_scaled @ R_icp.T + t_icp
    errors = np.linalg.norm(refined - rtk_pos, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))

    print(f"  ICP refinement: fitness={result.fitness:.4f}, RMSE={rmse:.6f}")

    return {
        'rotation': R_icp,
        'translation': t_icp,
        'rmse': rmse,
        'errors': errors,
        'aligned_positions': refined,
    }


def apply_alignment_to_full(sfm_pos_norm_full: np.ndarray,
                            scale: float,
                            R: np.ndarray,
                            t: np.ndarray,
                            icp_result: Optional[dict] = None) -> np.ndarray:
    """Apply the estimated transformation to ALL (non-subsampled) SfM positions.

    Args:
        sfm_pos_norm_full: (M, 3) all normalized SfM positions.
        scale: Scale factor from Horn's method.
        R: 3x3 rotation from Horn's method.
        t: 3x1 translation from Horn's method.
        icp_result: Optional ICP refinement result dict.

    Returns:
        (M, 3) aligned positions.
    """
    aligned = scale * (sfm_pos_norm_full @ R.T) + t

    if icp_result is not None:
        aligned = aligned @ icp_result['rotation'].T + icp_result['translation']

    return aligned


def denormalize_positions(positions_norm: np.ndarray,
                          first_position: np.ndarray,
                          first_rotation: np.ndarray) -> np.ndarray:
    """Reverse the first-pose normalization.

    Args:
        positions_norm: (N, 3) positions in normalized frame.
        first_position: Original first position.
        first_rotation: Original first rotation matrix (3x3).

    Returns:
        (N, 3) positions in the original coordinate frame.
    """
    return positions_norm @ first_rotation + first_position


def visualize_alignment(rtk_pos: np.ndarray,
                        sfm_before: np.ndarray,
                        sfm_after: np.ndarray,
                        output_path: str,
                        show_interactive: bool = False) -> None:
    """Create 3D visualization comparing trajectories before and after alignment.

    Args:
        rtk_pos: (N, 3) RTK ground truth positions.
        sfm_before: (M, 3) SfM positions before alignment.
        sfm_after: (M, 3) SfM positions after alignment.
        output_path: Output PNG path.
        show_interactive: If True, call plt.show().
    """
    fig = plt.figure(figsize=(14, 5))

    # Before alignment
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(rtk_pos[:, 0], rtk_pos[:, 1], rtk_pos[:, 2],
                c='blue', s=20, alpha=0.6, label='RTK (GT)')
    ax1.scatter(sfm_before[:, 0], sfm_before[:, 1], sfm_before[:, 2],
                c='red', s=30, alpha=0.6, label='SfM (before)')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('Before Alignment')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # After alignment
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(rtk_pos[:, 0], rtk_pos[:, 1], rtk_pos[:, 2],
                c='blue', s=20, alpha=0.6, label='RTK (GT)')
    ax2.scatter(sfm_after[:, 0], sfm_after[:, 1], sfm_after[:, 2],
                c='green', s=30, alpha=0.6, label='SfM (after)')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    ax2.set_title('After Alignment (Horn + Scale)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Saved visualization to: {output_path}")

    if show_interactive:
        plt.show()

    plt.close()


def generate_report(alignment: dict,
                    icp_result: Optional[dict],
                    rtk_count: int,
                    sfm_count: int,
                    output_path: str) -> None:
    """Write a text report of alignment results.

    Args:
        alignment: Results dict from align_trajectories().
        icp_result: Results from ICP refinement (or None).
        rtk_count: Number of RTK points used.
        sfm_count: Number of SfM points used.
        output_path: Output text file path.
    """
    with open(output_path, 'w') as f:
        f.write("=" * 60 + "\n")
        f.write("RTK to SfM Trajectory Alignment Report\n")
        f.write("=" * 60 + "\n\n")

        f.write("Data:\n")
        f.write(f"  RTK points: {rtk_count}\n")
        f.write(f"  SfM points: {sfm_count}\n\n")

        f.write("Horn's Method (with Umeyama scale):\n")
        f.write(f"  Scale factor: {alignment['scale']:.8f}\n")
        f.write(f"  RMSE: {alignment['rmse']:.6f} m\n")
        f.write(f"  Mean error: {np.mean(alignment['errors']):.6f} m\n")
        f.write(f"  Max error: {np.max(alignment['errors']):.6f} m\n")
        f.write(f"  Rotation:\n")
        for row in alignment['rotation']:
            f.write(f"    {row[0]:10.6f} {row[1]:10.6f} {row[2]:10.6f}\n")
        f.write(f"  Translation: [{alignment['translation'][0]:.6f}, "
                f"{alignment['translation'][1]:.6f}, "
                f"{alignment['translation'][2]:.6f}]\n\n")

        if icp_result is not None:
            f.write("ICP Refinement:\n")
            f.write(f"  RMSE: {icp_result['rmse']:.6f} m\n")
            f.write(f"  Mean error: {np.mean(icp_result['errors']):.6f} m\n")
            f.write(f"  Max error: {np.max(icp_result['errors']):.6f} m\n")

    print(f"Saved report to: {output_path}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description=(
            "Align SfM trajectory to RTK ground truth using Horn's method "
            "with scale estimation, optionally refined by ICP."
        )
    )
    parser.add_argument("rtk_csv", help="Path to RTK trajectory CSV")
    parser.add_argument("sfm_csv", help="Path to SfM trajectory CSV")
    parser.add_argument("--vis", action="store_true",
                        help="Show interactive 3D visualization")
    parser.add_argument("--icp", action="store_true",
                        help="Refine alignment with ICP (requires open3d)")
    parser.add_argument("--icp-max-dist", type=float, default=1.0,
                        help="Max correspondence distance for ICP (meters, default: 1.0)")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory (default: same as SfM CSV)")

    args = parser.parse_args()

    print("RTK to SfM Trajectory Alignment\n")

    # --- Resolve paths ---
    rtk_csv = Path(args.rtk_csv)
    sfm_csv = Path(args.sfm_csv)

    if not rtk_csv.exists():
        print(f"Error: RTK file not found: {rtk_csv}")
        sys.exit(1)
    if not sfm_csv.exists():
        print(f"Error: SfM file not found: {sfm_csv}")
        sys.exit(1)

    output_dir = Path(args.output_dir) if args.output_dir else sfm_csv.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    output_aligned = output_dir / f"{sfm_csv.stem}_aligned.csv"
    output_viz = output_dir / f"{sfm_csv.stem}_alignment_viz.png"
    output_report = output_dir / f"{sfm_csv.stem}_alignment_report.txt"

    # --- Load data ---
    rtk_df = load_trajectory(str(rtk_csv))
    sfm_df = load_trajectory(str(sfm_csv))

    # Remove RTK duplicates (static phase)
    rtk_df = remove_duplicates(rtk_df)

    # --- Extract positions and rotations ---
    rtk_pos, rtk_rot = extract_positions_and_rotations(rtk_df)
    sfm_pos, sfm_rot = extract_positions_and_rotations(sfm_df)

    # Save originals for denormalization later
    sfm_first_pos = sfm_pos[0].copy()
    sfm_first_rot = sfm_rot[0].copy()

    # --- Normalize both by first pose ---
    print("\nNormalizing trajectories (first pose -> origin)...")
    rtk_pos_norm, rtk_rot_norm = normalize_by_first_pose(rtk_pos, rtk_rot)
    sfm_pos_norm, sfm_rot_norm = normalize_by_first_pose(sfm_pos, sfm_rot)

    sfm_pos_norm_full = sfm_pos_norm.copy()  # keep all points for later

    # --- Subsample to equal length ---
    n_rtk = len(rtk_pos_norm)
    n_sfm = len(sfm_pos_norm)
    print(f"\nPoint counts: RTK={n_rtk}, SfM={n_sfm}")

    if n_rtk < n_sfm:
        print(f"  Downsampling SfM from {n_sfm} to {n_rtk}")
        sfm_pos_norm, _ = downsample_to_match(sfm_pos_norm, n_rtk)
    elif n_sfm < n_rtk:
        print(f"  Downsampling RTK from {n_rtk} to {n_sfm}")
        rtk_pos_norm, _ = downsample_to_match(rtk_pos_norm, n_sfm)

    # --- Align with Horn's method + scale ---
    print("\nRunning Horn's method with Umeyama scale estimation...")
    alignment = align_trajectories(sfm_pos_norm, rtk_pos_norm)

    # --- Optional ICP refinement ---
    icp_result = None
    if args.icp:
        print("\nRunning ICP refinement (rigid, no additional scale)...")
        icp_result = try_icp_refinement(
            alignment['aligned_positions'], rtk_pos_norm,
            max_distance=args.icp_max_dist,
        )

    # --- Apply to ALL SfM points ---
    print("\nApplying transformation to full SfM trajectory...")
    sfm_all_aligned_norm = apply_alignment_to_full(
        sfm_pos_norm_full,
        alignment['scale'], alignment['rotation'], alignment['translation'],
        icp_result=icp_result,
    )

    # Denormalize back to original RTK coordinate frame
    # (We want output in the RTK frame, so we denormalize using RTK's first pose)
    rtk_first_pos = rtk_pos[0].copy()
    rtk_first_rot = rtk_rot[0].copy()
    sfm_all_aligned = denormalize_positions(sfm_all_aligned_norm, rtk_first_pos, rtk_first_rot)

    # --- Save aligned trajectory ---
    sfm_df_aligned = sfm_df.copy()
    sfm_df_aligned[['x', 'y', 'z']] = sfm_all_aligned
    sfm_df_aligned.to_csv(output_aligned, index=False)
    print(f"Saved aligned SfM to: {output_aligned}")

    # --- Visualize ---
    visualize_alignment(rtk_pos, sfm_pos, sfm_all_aligned,
                        str(output_viz), show_interactive=args.vis)

    # --- Report ---
    generate_report(alignment, icp_result, len(rtk_pos), len(sfm_pos),
                    str(output_report))

    # --- Summary ---
    final_rmse = icp_result['rmse'] if icp_result else alignment['rmse']
    print(f"\nAlignment complete!")
    print(f"  Scale factor: {alignment['scale']:.6f}")
    print(f"  Final RMSE: {final_rmse:.6f} m")
    print(f"  Output files:")
    print(f"    {output_aligned}")
    print(f"    {output_viz}")
    print(f"    {output_report}")


if __name__ == "__main__":
    main()
