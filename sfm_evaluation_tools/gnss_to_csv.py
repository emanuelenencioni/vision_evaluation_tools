import sys
import csv
from pathlib import Path

# Import the high-level reader from the rosbags library
from rosbags.highlevel import AnyReader

# Scipy is excellent for handling 3D rotations
from scipy.spatial.transform import Rotation as R

def extract_odometry_to_csv(bag_path: str, odom_topic: str, output_csv_path: str):
    """
    Reads odometry messages from a ROS 2 bag file and saves the trajectory
    to a CSV file with a flattened 3x3 rotation matrix for orientation.

    Args:
        bag_path (str): The full path to the ROS 2 bag file.
        odom_topic (str): The specific odometry topic to read from the bag.
        output_csv_path (str): The full path for the output CSV file.
    """
    print(f"Opening bag file: {bag_path}")
    
    # Create a list to hold all our data rows
    trajectory_data = []

    # Use AnyReader to open the bag file
    with AnyReader([Path(bag_path)]) as reader:
        # Filter for connections on the specified odometry topic
        connections = [c for c in reader.connections if c.topic == odom_topic]
        if not connections:
            print(f"Error: Topic '{odom_topic}' not found in the bag file.")
            print("Available topics are:")
            for conn in reader.connections:
                print(f"  - {conn.topic} ({conn.msgtype})")
            return

        print(f"Reading messages from topic: '{odom_topic}'...")
        # Iterate through messages on the filtered connections
        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            # Deserialize the raw message data
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            # Extract position
            pos = msg.pose.pose.position
            
            # Extract quaternion
            quat = msg.pose.pose.orientation
            
            # --- Convert Quaternion to Rotation Matrix ---
            # Create a SciPy Rotation object from the quaternion
            rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
            
            # Get the 3x3 rotation matrix and flatten it into a 1D array of 9 elements
            rot_matrix = rotation.as_matrix()
            rot_matrix_flat = rot_matrix.flatten()
            
            # Combine timestamp into a single float value (seconds)
            timestamp_sec = float(timestamp_ns) / 1e9
            
            # Create the full data row: [timestamp, x, y, z, r11, r12, r13, ...]
            # The '*' unpacks the 9 elements from the flattened matrix
            row_data = [timestamp_sec, pos.x, pos.y, pos.z, *rot_matrix_flat]
            
            # Append the processed data to our list
            trajectory_data.append(row_data)

    if not trajectory_data:
        print("No odometry messages were processed. CSV file will not be created.")
        return

    # Write the collected data to a CSV file
    print(f"Writing {len(trajectory_data)} data points to {output_csv_path}...")
    try:
        with open(output_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            # --- MODIFICATION: Update the header row ---
            header = [
                'timestamp', 'x', 'y', 'z',
                'r11', 'r12', 'r13', 'r21', 'r22', 'r23', 'r31', 'r32', 'r33'
            ]
            writer.writerow(header)
            # Write all the data rows
            writer.writerows(trajectory_data)
        print("Successfully created CSV file.")
    except IOError as e:
        print(f"Error writing to CSV file: {e}")


if __name__ == "__main__":
    # --- How to run this script ---
    # python extract_trajectory.py <path_to_bag_file> <odometry_topic> <output_csv_path>
    
    if len(sys.argv) != 4:
        print("Usage: python extract_trajectory.py <path_to_bag_file> <odometry_topic> <output_csv_path>")
        sys.exit(1)
        
    bag_file = sys.argv[1]
    topic_name = sys.argv[2]
    csv_file = sys.argv[3]
    
    # Before running, ensure you have the necessary libraries installed:
    # pip install rosbags
    # pip install scipy
    
    extract_odometry_to_csv(bag_file, topic_name, csv_file)
