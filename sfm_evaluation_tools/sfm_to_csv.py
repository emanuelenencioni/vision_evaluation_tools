# -*- coding: utf-8 -*-
"""Extract Poses from SfM File to CSV

This script extracts all camera poses from a .sfm file (AliceVision structure from motion output)
and saves them to a CSV file with a flattened 3x3 rotation matrix for orientation.

The .sfm file is a JSON format containing pose information, views (images), and intrinsics.
"""

import json
import sys
import csv
import re
import numpy as np
from pathlib import Path


def extract_sfm_poses_to_csv(sfm_file_path: str, output_csv_path: str) -> None:
    """
    Extracts all camera poses from a .sfm file and saves them to a CSV file.

    Args:
        sfm_file_path (str): The full path to the .sfm file (JSON format).
        output_csv_path (str): The full path for the output CSV file.
    """
    print(f"Opening SfM file: {sfm_file_path}")

    try:
        with open(sfm_file_path, "r") as f:
            sfm_data = json.load(f)
    except FileNotFoundError:
        print(f"Error: SfM file '{sfm_file_path}' not found.")
        return
    except json.JSONDecodeError:
        print(f"Error: SfM file '{sfm_file_path}' is not valid JSON.")
        return

    assert sfm_data is not None, "SfM file is empty or invalid"

    # Extract poses and views
    poses = sfm_data.get("poses", [])
    views = sfm_data.get("views", [])

    if not poses:
        print("Warning: No poses found in SfM file.")
        return

    if not views:
        print("Warning: No views found in SfM file.")
        return

    print(f"Found {len(poses)} poses and {len(views)} views in SfM file")

    # Sort views by image filename so that the first frame (e.g. frame0000.jpg)
    # is truly the first entry. AliceVision .sfm files do NOT guarantee ordering
    # by frame number -- views are often stored by internal viewId.
    def _sort_key(view):
        """Extract a numeric sort key from the image filename.

        Handles common patterns like frame0000.jpg, IMG_0001.png, 00042.exr.
        Falls back to lexicographic sort on the full filename.
        """
        path = view.get("path", "")
        filename = Path(path).stem  # e.g. "frame0042"
        # Try to extract trailing digits (covers frame0042, IMG_0001, 00042, etc.)
        match = re.search(r"(\d+)$", filename)
        if match:
            return int(match.group(1))
        return filename  # lexicographic fallback

    views_sorted = sorted(views, key=_sort_key)

    first_view_path = Path(views_sorted[0].get("path", "")).name
    last_view_path = Path(views_sorted[-1].get("path", "")).name
    print(f"Views sorted by filename: first={first_view_path}, last={last_view_path}")

    # Create a mapping from poseId to pose data
    pose_by_id = {pose["poseId"]: pose for pose in poses}

    # Extract trajectory data: collect poses in sorted view order
    trajectory_data = []

    for view_idx, view in enumerate(views_sorted):
        pose_id = view.get("poseId")

        if pose_id is None or pose_id not in pose_by_id:
            continue

        pose = pose_by_id[pose_id]
        transform = pose["pose"]["transform"]

        # Extract position (center)
        center = transform["center"]
        x, y, z = float(center[0]), float(center[1]), float(center[2])

        # Extract rotation matrix (stored as flattened 9 elements)
        rotation_flat = transform["rotation"]
        rotation_matrix = np.array(rotation_flat).reshape(3, 3)

        # Flatten the rotation matrix to 9 elements
        rot_matrix_flat = rotation_matrix.flatten().tolist()

        # Use sorted view index as timestamp
        timestamp = float(view_idx)

        # Store the image filename for traceability
        image_path = view.get("path", "")
        image_name = Path(image_path).name

        # Create the data row: [timestamp, x, y, z, r11..r33, image_name]
        row_data = [timestamp, x, y, z] + rot_matrix_flat + [image_name]

        trajectory_data.append(row_data)

    if not trajectory_data:
        print("Error: No valid pose data was extracted from the SfM file.")
        return

    # Write the collected data to a CSV file
    print(f"Writing {len(trajectory_data)} poses to {output_csv_path}...")
    try:
        with open(output_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            # Write header row
            header = [
                "timestamp",
                "x",
                "y",
                "z",
                "r11",
                "r12",
                "r13",
                "r21",
                "r22",
                "r23",
                "r31",
                "r32",
                "r33",
                "image_name",
            ]
            writer.writerow(header)
            # Write all data rows
            writer.writerows(trajectory_data)
        print("Successfully created CSV file.")
    except IOError as e:
        print(f"Error writing to CSV file: {e}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python sfm_to_csv.py <path_to_sfm_file> <output_csv_path>")
        print("Example: python sfm_to_csv.py cameras.sfm poses.csv")
        sys.exit(1)

    sfm_file = sys.argv[1]
    csv_file = sys.argv[2]

    extract_sfm_poses_to_csv(sfm_file, csv_file)
