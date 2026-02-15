import numpy as np



def subsample(signal: np.ndarray, new_length: int):
    if new_length >= len(signal):
        return signal
    indices = np.round(np.linspace(0, len(signal) - 1, new_length)).astype(int)
    return signal[indices]


def find_nearest_neighbors(source_points: np.ndarray, target_points: np.ndarray):
    """
    Finds the nearest neighbor in the target point cloud for each point
    in the source point cloud.

    This function establishes the correspondences needed before applying
    Horn's method.

    Args:
        source_points (np.ndarray): The source point cloud, shape (N, 3).
        target_points (np.ndarray): The target point cloud, shape (M, 3).

    Returns:
        np.ndarray: A subset of the source points, shape (N, 3).
        np.ndarray: The corresponding points from the target cloud, shape (N, 3).
    """
    corresponding_target_points = np.zeros_like(source_points)

    for i, p1 in enumerate(source_points):
        # Calculate squared Euclidean distances from p1 to all points in target_points
        distances = np.sum((target_points - p1)**2, axis=1)
        # Find the index of the minimum distance
        nearest_neighbor_index = np.argmin(distances)
        # Assign the corresponding point
        corresponding_target_points[i] = target_points[nearest_neighbor_index]
        
    return source_points, corresponding_target_points


def horn_method(p1: np.ndarray, p2: np.ndarray):
    """
    Applies Horn's method (quaternion-based) to find the optimal rigid
    transformation between two sets of corresponding 3D points.

    Args:
        p1 (np.ndarray): The first set of points, shape (N, 3).
        p2 (np.ndarray): The second set of corresponding points, shape (N, 3).

    Returns:
        np.ndarray: The 3x3 rotation matrix.
        np.ndarray: The 3x1 translation vector.
    """
    # 1. Calculate the centroids of both point clouds
    centroid1 = np.mean(p1, axis=0)
    centroid2 = np.mean(p2, axis=0)

    # 2. Center the points by subtracting the centroids
    p1_centered = p1 - centroid1
    p2_centered = p2 - centroid2

    # 3. Calculate the covariance matrix H
    H = np.dot(p1_centered.T, p2_centered)
    
    # Unpack the elements of H for the symmetric matrix N
    Sxx, Sxy, Sxz = H[0,0], H[0,1], H[0,2]
    Syx, Syy, Syz = H[1,0], H[1,1], H[1,2]
    Szx, Szy, Szz = H[2,0], H[2,1], H[2,2]

    # 4. Form the 4x4 symmetric matrix N
    N = np.array([
        [Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx],
        [Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz],
        [Szx - Sxz, Sxy + Syx, Syy - Sxx - Szz, Syz + Szy],
        [Sxy - Syx, Szx + Sxz, Syz + Szy, Szz - Sxx - Syy]
    ])

    # 5. Find the eigenvector corresponding to the largest eigenvalue of N
    eigenvalues, eigenvectors = np.linalg.eig(N)
    max_eigenvalue_index = np.argmax(eigenvalues)
    q = eigenvectors[:, max_eigenvalue_index] # This is our optimal quaternion [q0, q1, q2, q3]

    # 6. Convert the quaternion to a rotation matrix
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    
    R = np.array([
        [q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]
    ])

    # 7. Calculate the translation vector
    t = centroid2.T - np.dot(R, centroid1.T)

    return R, t


def horn_method_with_scale(p1: np.ndarray, p2: np.ndarray):
    """
    Applies Horn's method with Umeyama's scale estimation to find the optimal
    similarity transformation (rotation + translation + uniform scale) between
    two sets of corresponding 3D points.

    Solves: p2 = s * R @ p1 + t

    Reference: Umeyama, "Least-Squares Estimation of Transformation Parameters
    Between Two Point Patterns", IEEE PAMI 1991.

    Args:
        p1 (np.ndarray): Source points, shape (N, 3). Will be scaled.
        p2 (np.ndarray): Target points, shape (N, 3). Reference scale.

    Returns:
        np.ndarray: The 3x3 rotation matrix R.
        np.ndarray: The 3x1 translation vector t.
        float: The uniform scale factor s.
    """
    assert len(p1) == len(p2), f"Point sets must have equal length, got {len(p1)} and {len(p2)}"
    assert len(p1) >= 3, f"Need at least 3 point pairs, got {len(p1)}"

    # 1. Compute centroids
    centroid1 = np.mean(p1, axis=0)
    centroid2 = np.mean(p2, axis=0)

    # 2. Center the points
    p1_centered = p1 - centroid1
    p2_centered = p2 - centroid2

    # 3. Compute variance of source points (for scale estimation)
    variance_p1 = np.mean(np.sum(p1_centered ** 2, axis=1))

    # 4. Compute covariance matrix H and find rotation via SVD
    #    (SVD is more numerically stable than the quaternion eigenvector approach
    #     when scale is involved)
    H = p1_centered.T @ p2_centered / len(p1)

    U, S, Vt = np.linalg.svd(H)

    # Handle reflection case: ensure proper rotation (det(R) = +1)
    d = np.linalg.det(Vt.T @ U.T)
    sign_matrix = np.diag([1.0, 1.0, np.sign(d)])

    R = Vt.T @ sign_matrix @ U.T

    # 5. Compute scale (Umeyama formula)
    #    s = trace(S * sign_matrix) / variance_p1
    scale = np.trace(np.diag(S) @ sign_matrix) / variance_p1

    # 6. Compute translation: t = centroid2 - s * R @ centroid1
    t = centroid2 - scale * R @ centroid1

    return R, t, scale


def compute_path_length(positions: np.ndarray) -> float:
    """
    Compute total path length of a trajectory (sum of consecutive segment lengths).

    Args:
        positions (np.ndarray): Ordered positions, shape (N, 3).

    Returns:
        float: Total path length in the same units as positions.
    """
    if len(positions) < 2:
        return 0.0
    segments = np.diff(positions, axis=0)
    segment_lengths = np.linalg.norm(segments, axis=1)
    return float(np.sum(segment_lengths))


def register_point_clouds(source_points: np.ndarray, target_points: np.ndarray):
    """
    A complete registration pipeline for two point clouds of potentially
    different lengths.

    Args:
        source_points (np.ndarray): The point cloud to be aligned.
        target_points (np.ndarray): The reference point cloud.

    Returns:
        np.ndarray: The 3x3 rotation matrix.
        np.ndarray: The 3x1 translation vector.
    """
    # Step 1: Find corresponding points using nearest neighbors
    p1, p2 = find_nearest_neighbors(source_points, target_points)
    
    # Step 2: Apply Horn's method to the corresponding points
    R, t = horn_method(p1, p2)
    
    return R, t


# --- Example Usage ---
if __name__ == '__main__':
    # 1. Create a "target" point cloud (a simple cube)
    # This cloud will have more points.
    target = np.array([
        [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
        [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1],
        [0.5, 0.5, 0], [0.5, 0, 0.5], [0, 0.5, 0.5] # Extra points
    ])

    # 2. Define a ground truth transformation
    true_angle_y = np.pi / 4  # 45 degrees rotation around Y-axis
    true_R = np.array([
        [np.cos(true_angle_y), 0, np.sin(true_angle_y)],
        [0, 1, 0],
        [-np.sin(true_angle_y), 0, np.cos(true_angle_y)]
    ])
    true_t = np.array([5, -3, 2])

    # 3. Create the "source" point cloud by transforming a subset of the target
    # This cloud will have fewer points.
    source_subset = np.array([
        [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
        [0, 0, 1], [1, 0, 1]
    ])
    source = np.dot(source_subset, true_R.T) + true_t
    # Add some noise to make it more realistic
    source += np.random.randn(source.shape[0], 3) * 0.01

    print("--- Problem Setup ---")
    print(f"Source cloud has {source.shape[0]} points.")
    print(f"Target cloud has {target.shape[0]} points.")
    print("\n--- Running Registration ---")

    # 4. Run the full registration pipeline
    estimated_R, estimated_t = register_point_clouds(source, target)
    
    # 5. Print the results
    print("\n--- Results ---")
    print("Ground Truth Rotation Matrix:\n", true_R)
    print("\nEstimated Rotation Matrix:\n", estimated_R)
    print("\nGround Truth Translation Vector:\n", true_t)
    print("\nEstimated Translation Vector:\n", estimated_t)

    # 6. Apply the transformation to see how well it aligns
    source_aligned = np.dot(source, estimated_R.T) + estimated_t

    # You can now compare 'source_aligned' with 'target'
    # For a quantitative measure, you can calculate the Mean Squared Error (MSE)
    # between the aligned points and their nearest neighbors in the target cloud.
    _, corresponding_target = find_nearest_neighbors(source_aligned, target)
    mse = np.mean(np.sum((source_aligned - corresponding_target)**2, axis=1))
    print(f"\nMean Squared Error (MSE) after alignment: {mse:.6f}")
