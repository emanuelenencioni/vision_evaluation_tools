"""Extract RTK/GPS Trajectory from mixer_msgs

This script extracts RTK and GPS trajectory data from ROS2 bag files containing
mixer_msgs/Rtk and mixer_msgs/Gps messages. It converts spherical GPS coordinates
to local Cartesian ENU (East/North/Up) coordinates, merges with relative RTK data,
and optionally visualizes the trajectory with fix status indicators.

The output CSV includes position, rotation matrices (identity - no heading info),
fix status (RTK_FIXED=4 or RTK_FLOAT=3), and accuracy estimates.

Usage:
    python rtk_gnss_to_csv.py <bag_path> <rtk_topic> <gps_topic> <output_csv> [OPTIONS]
    
    python rtk_gnss_to_csv.py flight.bag /rtk /gps trajectory.csv --vis
    python rtk_gnss_to_csv.py flight.bag /rtk /gps trajectory.csv --vis --vis-output plot.png
"""

import sys
import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, get_typestore, Stores
from rosbags.typesys.store import Typestore
import pyproj
from typing import Dict, List, Tuple, Optional


def setup_typestore_with_mixer_msgs(ros2_ws_path: Optional[str] = None) -> Typestore:
    """
    Create typestore with support for mixer_msgs custom types.
    
    Starts with default typestore (includes std_msgs, sensor_msgs, etc),
    then parses and registers mixer_msgs definitions from ROS2 workspace.
    
    Args:
        ros2_ws_path: Optional path to ROS2 workspace (e.g., ~/Documenti/ros2_ws)
                     If None, will prompt user for path
    
    Returns:
        Typestore object with custom message types loaded
    """
    # Start with default typestore that includes all standard ROS types
    typestore = get_typestore(Stores.LATEST)
    
    # Ask user for workspace path if not provided
    if ros2_ws_path is None:
        ros2_ws_path = input(
            "Enter ROS2 workspace path (default: ~/Documenti/ros2_ws): "
        ).strip()
        if not ros2_ws_path:
            ros2_ws_path = "~/Documenti/ros2_ws"
    
    workspace_path = Path(ros2_ws_path).expanduser()
    msg_dir = workspace_path / "src" / "mixer_msgs" / "msg"
    
    if not msg_dir.exists():
        print(f"⚠ mixer_msgs definitions not found at {msg_dir}")
        print("  Proceeding with default typestore (no mixer_msgs support)...")
        return typestore
    
    # Parse and register mixer_msgs definitions
    try:
        # Read Rtk.msg and Gps.msg
        rtk_file = msg_dir / "Rtk.msg"
        gps_file = msg_dir / "Gps.msg"
        
        if rtk_file.exists() and gps_file.exists():
            with open(rtk_file, 'r') as f:
                rtk_def = f.read()
            with open(gps_file, 'r') as f:
                gps_def = f.read()
            
            # Parse message definitions into typesdict format
            rtk_types = get_types_from_msg(rtk_def, 'mixer_msgs/Rtk')
            gps_types = get_types_from_msg(gps_def, 'mixer_msgs/Gps')
            
            # Merge and register with typestore
            all_types = {**rtk_types, **gps_types}
            typestore.register(all_types)
            print("✓ Successfully registered mixer_msgs types with typestore")
        else:
            print(f"⚠ Could not find mixer_msgs definition files")
            print("  Proceeding with default typestore...")
    
    except Exception as e:
        print(f"⚠ Error registering mixer_msgs types: {type(e).__name__}: {e}")
        print("  Proceeding with default typestore...")
    
    return typestore


def read_rtk_gps_bag(bag_path: str, rtk_topic: str, gps_topic: str, typestore: Typestore) -> Tuple[List[Dict], List[Dict]]:
    """
    Read RTK and GPS messages from ROS2 bag file containing custom mixer_msgs.
    
    Args:
        bag_path: Path to ROS2 bag file
        rtk_topic: RTK message topic name
        gps_topic: GPS message topic name
        typestore: Typestore with custom message definitions
    
    Returns:
        Tuple of (rtk_data_list, gps_data_list) containing message dictionaries
    """
    print(f"Opening bag file: {bag_path}")
    
    rtk_data = []
    gps_data = []
    
    try:
        # Use AnyReader with custom typestore
        with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
            # Get all connections
            all_topics = [conn.topic for conn in reader.connections]
            print(f"Available topics: {all_topics}")
            
            # Read RTK messages
            rtk_connections = [c for c in reader.connections if c.topic == rtk_topic]
            if rtk_connections:
                print(f"Reading RTK messages from '{rtk_topic}'...")
                for connection, timestamp_ns, rawdata in reader.messages(connections=rtk_connections):
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    # Note: relpos* fields are in centimeters, acc* fields are in centimeters
                    rtk_data.append({
                        'timestamp_ns': timestamp_ns,
                        'timestamp_sec': float(timestamp_ns) / 1e9,
                        'relposn': msg.relposn.data / 100.0,  # Convert cm -> m (North)
                        'relpose': msg.relpose.data / 100.0,  # Convert cm -> m (East)
                        'relposd': msg.relposd.data / 100.0,  # Convert cm -> m (Down)
                        'accn': msg.accn.data / 100.0,  # Convert cm -> m
                        'acce': msg.acce.data / 100.0,  # Convert cm -> m
                        'accd': msg.accd.data / 100.0,  # Convert cm -> m
                        'veln': msg.veln.data,
                        'vele': msg.vele.data,
                        'veld': msg.veld.data,
                        'tow': msg.tow.data,
                        'gps_fix_1': msg.gps_fix_1.data,
                        'gps_fix_2': msg.gps_fix_2.data,
                    })
                print(f"  Read {len(rtk_data)} RTK messages")
            else:
                print(f"  Warning: RTK topic '{rtk_topic}' not found")
            
            # Read GPS messages
            gps_connections = [c for c in reader.connections if c.topic == gps_topic]
            if gps_connections:
                print(f"Reading GPS messages from '{gps_topic}'...")
                for connection, timestamp_ns, rawdata in reader.messages(connections=gps_connections):
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    # Note: gpsaltitude is in centimeters, eph/epv are in millimeters
                    gps_data.append({
                        'timestamp_ns': timestamp_ns,
                        'timestamp_sec': float(timestamp_ns) / 1e9,
                        'latitude': msg.gpslatitude.data,
                        'longitude': msg.gpslongitude.data,
                        'altitude': msg.gpsaltitude.data / 100.0,  # Convert cm -> m
                        'eph': msg.eph.data / 1000.0,  # Convert mm -> m (horizontal accuracy)
                        'epv': msg.epv.data / 1000.0,  # Convert mm -> m (vertical accuracy)
                        'tow': msg.tow.data,
                        'gps_fix_1': msg.gps_fix_1.data,
                        'gps_fix_2': msg.gps_fix_2.data,
                    })
                print(f"  Read {len(gps_data)} GPS messages")
            else:
                print(f"  Warning: GPS topic '{gps_topic}' not found")
    
    except Exception as e:
        print(f"Error reading bag file: {e}")
        sys.exit(1)
    
    return rtk_data, gps_data


def find_first_valid_fix(rtk_data: List[Dict], gps_data: List[Dict]) -> Optional[Dict]:
    """
    Find first measurement with fix status 4 (RTK_FIXED), fallback to 3 (RTK_FLOAT).
    
    Args:
        rtk_data: RTK messages list
        gps_data: GPS messages list
    
    Returns:
        Dictionary with lat, lon, alt of first valid fix, or None if not found
    """
    # Priority 1: First RTK message with fix 4
    for rtk_msg in rtk_data:
        if rtk_msg['gps_fix_1'] == 4 or rtk_msg['gps_fix_2'] == 4:
            # Use corresponding GPS message for absolute coordinates
            for gps_msg in gps_data:
                if abs(gps_msg['timestamp_sec'] - rtk_msg['timestamp_sec']) < 0.1:
                    return {
                        'latitude': gps_msg['latitude'],
                        'longitude': gps_msg['longitude'],
                        'altitude': gps_msg['altitude']
                    }
    
    # Priority 2: First message with fix 3
    for rtk_msg in rtk_data:
        if rtk_msg['gps_fix_1'] == 3 or rtk_msg['gps_fix_2'] == 3:
            for gps_msg in gps_data:
                if abs(gps_msg['timestamp_sec'] - rtk_msg['timestamp_sec']) < 0.1:
                    return {
                        'latitude': gps_msg['latitude'],
                        'longitude': gps_msg['longitude'],
                        'altitude': gps_msg['altitude']
                    }
    
    # Fallback: use first available GPS message
    if gps_data:
        return {
            'latitude': gps_data[0]['latitude'],
            'longitude': gps_data[0]['longitude'],
            'altitude': gps_data[0]['altitude']
        }
    
    return None


def convert_gps_to_enu(lat: float, lon: float, alt: float, ref_lat: float, ref_lon: float, ref_alt: float) -> np.ndarray:
    """
    Convert GPS spherical coordinates to local ENU (East/North/Up) Cartesian.
    
    Args:
        lat, lon, alt: Target coordinates (degrees, degrees, meters)
        ref_lat, ref_lon, ref_alt: Reference point for ENU origin
    
    Returns:
        NumPy array [east, north, up] in meters
    """
    # Use pyproj for geodetic conversions
    # WGS84 geographic (lat/lon)
    geodetic = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    
    # Local transverse Mercator projection centered at reference point
    # This gives us local (x, y) coordinates in meters
    local_proj = pyproj.Proj(proj='tmerc', lat_0=ref_lat, lon_0=ref_lon, ellps='WGS84')
    
    transformer = pyproj.Transformer.from_proj(geodetic, local_proj, always_xy=True)
    
    # Convert reference point to local coords (origin) - should be (0, 0)
    ref_x, ref_y = transformer.transform(ref_lon, ref_lat)
    
    # Convert target point
    target_x, target_y = transformer.transform(lon, lat)
    
    # Calculate ENU coordinates
    # Transverse Mercator: x=East, y=North (approximately for small areas)
    east = target_x - ref_x
    north = target_y - ref_y
    up = alt - ref_alt  # Relative altitude
    
    return np.array([east, north, up])


def merge_rtk_gps_data(rtk_data: List[Dict], gps_data: List[Dict], time_tolerance_sec: float = 0.01) -> List[Dict]:
    """
    Merge RTK and GPS data by timestamp.
    
    Args:
        rtk_data: RTK messages
        gps_data: GPS messages
        time_tolerance_sec: Timestamp matching tolerance in seconds
    
    Returns:
        List of merged measurements
    """
    merged = []
    
    for rtk_msg in rtk_data:
        # Find closest GPS message
        closest_gps = None
        min_time_diff = float('inf')
        
        for gps_msg in gps_data:
            time_diff = abs(gps_msg['timestamp_sec'] - rtk_msg['timestamp_sec'])
            if time_diff < min_time_diff and time_diff < time_tolerance_sec:
                min_time_diff = time_diff
                closest_gps = gps_msg
        
        # Create merged entry
        merged_entry = {
            'timestamp_sec': rtk_msg['timestamp_sec'],
            'relposn': rtk_msg['relposn'],  # Relative North
            'relpose': rtk_msg['relpose'],  # Relative East
            'relposd': rtk_msg['relposd'],  # Relative Down
            'accn': rtk_msg['accn'],
            'acce': rtk_msg['acce'],
            'accd': rtk_msg['accd'],
            'gps_fix': rtk_msg['gps_fix_1'] if rtk_msg['gps_fix_1'] != 0 else rtk_msg['gps_fix_2'],
            'has_gps': closest_gps is not None,
        }
        
        if closest_gps:
            merged_entry.update({
                'latitude': closest_gps['latitude'],
                'longitude': closest_gps['longitude'],
                'altitude': closest_gps['altitude'],
                'eph': closest_gps['eph'],
                'epv': closest_gps['epv'],
            })
        
        merged.append(merged_entry)
    
    return merged


def extract_rtk_gps_trajectory(
    bag_path: str,
    rtk_topic: str,
    gps_topic: str,
    output_csv_path: str,
    typestore: Typestore,
    visualize: bool = False,
    vis_output: Optional[str] = None
) -> List[Dict]:
    """
    Extract RTK/GPS trajectory from ROS2 bag and optionally visualize.
    
    Args:
        bag_path: Path to ROS2 bag file
        rtk_topic: RTK message topic
        gps_topic: GPS message topic
        output_csv_path: Output CSV file path
        typestore: Typestore with custom message types
        visualize: Enable visualization
        vis_output: Optional visualization output filename
    
    Returns:
        List of trajectory dictionaries with Cartesian coordinates
    """
    # Read raw data from bag
    rtk_data, gps_data = read_rtk_gps_bag(bag_path, rtk_topic, gps_topic, typestore)
    
    if not gps_data:
        print("Error: No GPS data found in bag. Cannot extract trajectory.")
        sys.exit(1)
    
    # Find reference point (first valid fix)
    ref_point = find_first_valid_fix(rtk_data, gps_data)
    if not ref_point:
        print("Error: No valid GPS fix found. Cannot establish reference point.")
        sys.exit(1)
    
    print(f"Using reference point: lat={ref_point['latitude']:.8f}, lon={ref_point['longitude']:.8f}, alt={ref_point['altitude']:.2f}")
    
    # Merge RTK and GPS data
    merged_data = merge_rtk_gps_data(rtk_data, gps_data)
    
    # Convert to Cartesian and build trajectory
    trajectory_data = []
    fix_counts = {'4': 0, '3': 0, 'invalid': 0}
    
    for measurement in merged_data:
        if not measurement['has_gps']:
            continue
        
        # Validate fix status (should be 3=float or 4=fixed)
        if measurement['gps_fix'] not in [3, 4]:
            fix_counts['invalid'] += 1
            continue
        
        # Validate accuracy values (should be reasonable, <10m for RTK)
        if measurement['accn'] > 10.0 or measurement['acce'] > 10.0 or measurement['accd'] > 10.0:
            fix_counts['invalid'] += 1
            continue
        
        # Use RTK relative position (much more accurate than GPS alone)
        # RTK provides relative position from base station
        # Interpretation: relpose=East, relposn=North, relposd=Down
        east = measurement['relpose']
        north = measurement['relposn']
        down = measurement['relposd']
        
        # If we had a reference point, we would convert to global ENU
        # For now, use RTK relative coordinates directly (base station is origin)
        enu_coords = np.array([east, north, -down])
        
        # Determine fix status string
        fix_status = "RTK_FIXED" if measurement['gps_fix'] == 4 else "RTK_FLOAT"
        if measurement['gps_fix'] == 4:
            fix_counts['4'] += 1
        elif measurement['gps_fix'] == 3:
            fix_counts['3'] += 1
        
        # Identity rotation matrix (no heading info from GNSS)
        rot_matrix = np.eye(3).flatten()
        
        # Build trajectory point
        traj_point = {
            'timestamp': measurement['timestamp_sec'],
            'x': enu_coords[0],
            'y': enu_coords[1],
            'z': enu_coords[2],
            'rot_matrix': rot_matrix,
            'fix_status': fix_status,
            'accuracy_n': measurement['accn'],
            'accuracy_e': measurement['acce'],
            'accuracy_d': measurement['accd'],
        }
        
        trajectory_data.append(traj_point)
    
    print(f"Extracted {len(trajectory_data)} trajectory points")
    if len(trajectory_data) > 0:
        print(f"  Fix 4 (RTK_FIXED): {fix_counts['4']} ({100*fix_counts['4']/len(trajectory_data):.1f}%)")
        print(f"  Fix 3 (RTK_FLOAT): {fix_counts['3']} ({100*fix_counts['3']/len(trajectory_data):.1f}%)")
    if fix_counts['invalid'] > 0:
        print(f"  Invalid/filtered: {fix_counts['invalid']}")
    
    # Write CSV
    write_trajectory_to_csv(trajectory_data, output_csv_path)
    
    # Optionally visualize
    if visualize:
        visualize_trajectory(trajectory_data, vis_output)
    
    return trajectory_data


def write_trajectory_to_csv(trajectory_data: List[Dict], output_csv_path: str) -> None:
    """
    Write trajectory data to CSV file.
    
    Args:
        trajectory_data: List of trajectory dictionaries
        output_csv_path: Output file path
    """
    print(f"Writing {len(trajectory_data)} data points to {output_csv_path}...")
    
    try:
        with open(output_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Header row
            header = [
                'timestamp', 'x', 'y', 'z',
                'r11', 'r12', 'r13', 'r21', 'r22', 'r23', 'r31', 'r32', 'r33',
                'fix_status', 'accuracy_n', 'accuracy_e', 'accuracy_d'
            ]
            writer.writerow(header)
            
            # Data rows
            for point in trajectory_data:
                row = [
                    point['timestamp'],
                    point['x'], point['y'], point['z'],
                    *point['rot_matrix'],
                    point['fix_status'],
                    point['accuracy_n'], point['accuracy_e'], point['accuracy_d']
                ]
                writer.writerow(row)
        
        print("Successfully created CSV file.")
    except IOError as e:
        print(f"Error writing to CSV file: {e}")
        sys.exit(1)


def visualize_trajectory(trajectory_data: List[Dict], output_filename: Optional[str] = None) -> None:
    """
    Visualize RTK/GPS trajectory with 2D and 3D views.
    
    Args:
        trajectory_data: List of trajectory dictionaries
        output_filename: Optional filename to save plot (PNG format)
    """
    if not trajectory_data:
        print("No trajectory data to visualize.")
        return
    
    # Extract data
    x = np.array([p['x'] for p in trajectory_data])
    y = np.array([p['y'] for p in trajectory_data])
    z = np.array([p['z'] for p in trajectory_data])
    fix_status = [p['fix_status'] for p in trajectory_data]
    accuracy_n = np.array([p['accuracy_n'] for p in trajectory_data])
    accuracy_e = np.array([p['accuracy_e'] for p in trajectory_data])
    accuracy_d = np.array([p['accuracy_d'] for p in trajectory_data])
    
    # Separate by fix status
    fix4_mask = np.array([s == 'RTK_FIXED' for s in fix_status])
    fix3_mask = np.array([s == 'RTK_FLOAT' for s in fix_status])
    
    fix4_count = np.sum(fix4_mask)
    fix3_count = np.sum(fix3_mask)
    
    # Create figure with 2D and 3D subplots
    fig = plt.figure(figsize=(15, 6))
    
    # 2D plot (top-down view)
    ax1 = fig.add_subplot(121)
    if fix4_count > 0:
        ax1.scatter(x[fix4_mask], y[fix4_mask], c='green', s=20, label=f'Fix 4 ({fix4_count})', alpha=0.7)
    if fix3_count > 0:
        ax1.scatter(x[fix3_mask], y[fix3_mask], c='orange', s=20, label=f'Fix 3 ({fix3_count})', alpha=0.7)
    
    # Plot trajectory line
    ax1.plot(x, y, 'b-', alpha=0.3, linewidth=0.8)
    
    # Start and end markers
    ax1.plot(x[0], y[0], 'g*', markersize=20, label='Start', zorder=5)
    ax1.plot(x[-1], y[-1], 'rs', markersize=12, label='End', zorder=5)
    
    ax1.set_xlabel('East (m)')
    ax1.set_ylabel('North (m)')
    ax1.set_title('RTK/GPS Trajectory (Top-Down View)')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    
    # 3D plot
    ax2 = fig.add_subplot(122, projection='3d')
    if fix4_count > 0:
        ax2.scatter(x[fix4_mask], y[fix4_mask], z[fix4_mask], c='green', s=20, label=f'Fix 4 ({fix4_count})', alpha=0.7)
    if fix3_count > 0:
        ax2.scatter(x[fix3_mask], y[fix3_mask], z[fix3_mask], c='orange', s=20, label=f'Fix 3 ({fix3_count})', alpha=0.7)
    
    # Plot trajectory line
    ax2.plot(x, y, z, 'b-', alpha=0.3, linewidth=0.8)
    
    # Start and end markers
    ax2.plot([x[0]], [y[0]], [z[0]], 'g*', markersize=15, label='Start', zorder=5)
    ax2.plot([x[-1]], [y[-1]], [z[-1]], 'rs', markersize=10, label='End', zorder=5)
    
    ax2.set_xlabel('East (m)')
    ax2.set_ylabel('North (m)')
    ax2.set_zlabel('Up (m)')
    ax2.set_title('RTK/GPS Trajectory (3D View)')
    ax2.legend()
    
    # Add metadata text box
    metadata_text = (
        f'Trajectory Summary\n'
        f'─────────────────\n'
        f'Total Points: {len(trajectory_data)}\n'
        f'Fix 4 (RTK): {fix4_count} ({100*fix4_count/len(trajectory_data):.1f}%)\n'
        f'Fix 3 (RTK): {fix3_count} ({100*fix3_count/len(trajectory_data):.1f}%)\n'
        f'\n'
        f'Bounds:\n'
        f'  E: {np.min(x):.2f} to {np.max(x):.2f} m\n'
        f'  N: {np.min(y):.2f} to {np.max(y):.2f} m\n'
        f'  U: {np.min(z):.2f} to {np.max(z):.2f} m\n'
        f'\n'
        f'Accuracy (mean ± std):\n'
        f'  N: {np.mean(accuracy_n):.6f} ± {np.std(accuracy_n):.6f} m\n'
        f'  E: {np.mean(accuracy_e):.6f} ± {np.std(accuracy_e):.6f} m\n'
        f'  D: {np.mean(accuracy_d):.6f} ± {np.std(accuracy_d):.6f} m'
    )
    
    fig.text(0.02, 0.98, metadata_text, transform=fig.transFigure, fontsize=9,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Save or show
    if output_filename:
        print(f"Saving visualization to {output_filename}...")
        plt.savefig(output_filename, dpi=150, bbox_inches='tight')
        print("Visualization saved.")
    
    print("Displaying visualization...")
    plt.show()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Extract RTK/GPS trajectory from ROS2 bag with mixer_msgs"
    )
    
    parser.add_argument('bag_path', help='Path to ROS2 bag file')
    parser.add_argument('rtk_topic', help='RTK message topic name (e.g., /mixer/telemetry/rtk)')
    parser.add_argument('gps_topic', help='GPS message topic name (e.g., /mixer/telemetry/gps)')
    parser.add_argument('output_csv', help='Output CSV file path')
    parser.add_argument('--vis', '--visualize', action='store_true', dest='visualize',
                        help='Enable trajectory visualization')
    parser.add_argument('--vis-output', dest='vis_output', default=None,
                        help='Visualization output filename (PNG format)')
    parser.add_argument('--ros2-ws', dest='ros2_ws', default=None,
                        help='Path to ROS2 workspace (will prompt if not specified)')
    
    args = parser.parse_args()
    
    # Setup typestore for mixer_msgs custom types
    print("Preparing typestore for mixer_msgs custom types...")
    typestore = setup_typestore_with_mixer_msgs(args.ros2_ws)
    
    # Validate inputs
    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f"Error: Bag file '{bag_path}' not found.")
        sys.exit(1)
    
    # Extract trajectory
    extract_rtk_gps_trajectory(
        str(bag_path),
        args.rtk_topic,
        args.gps_topic,
        args.output_csv,
        typestore=typestore,
        visualize=args.visualize,
        vis_output=args.vis_output
    )


if __name__ == "__main__":
    main()
