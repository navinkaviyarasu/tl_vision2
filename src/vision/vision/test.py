import numpy as np
from scipy.spatial.transform import Rotation as R

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) in degrees to a quaternion.
    The order of rotations is ZYX (yaw, pitch, roll).
    """
    # Convert Euler angles to a rotation object
    r = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
    return r.as_quat()  # Return quaternion as [x, y, z, w]
    return r.as_quat(scalar_first=True)  # Return quaternion as [w, x, y, z]

def nwu_to_ned(position_nwu, velocity_nwu, orientation_nwu):
    """
    Convert position, velocity, and orientation from NWU frame to NED frame.
    The orientation here is assumed to be a quaternion.
    """
    # Define the rotation matrix to convert NWU to NED
    R_nwu_to_ned = np.array([[1, 0, 0],
                              [0, -1, 0],
                              [0, 0, -1]])

    # Apply the rotation to position and velocity
    position_ned = np.dot(R_nwu_to_ned, position_nwu)
    velocity_ned = np.dot(R_nwu_to_ned, velocity_nwu)

    # Apply the same rotation matrix to the orientation quaternion
    r = R.from_matrix(R_nwu_to_ned)
    orientation_ned = r.apply(orientation_nwu)

    return position_ned, velocity_ned, orientation_ned

def apply_sensor_orientation(position_ned, velocity_ned, orientation_ned, mounting_quaternion):
    """
    Apply the sensor mounting orientation (given by a quaternion) to the data.
    The mounting orientation is provided in the FRD frame, and is applied to the NED data.
    """
    # Create a rotation object from the mounting quaternion
    r_sensor_frd = R.from_quat(mounting_quaternion)
    
    # Apply the sensor's mounting orientation to position, velocity, and orientation
    position_transformed = r_sensor_frd.apply(position_ned)
    velocity_transformed = r_sensor_frd.apply(velocity_ned)
    orientation_transformed = r_sensor_frd.apply(orientation_ned)

    return position_transformed, velocity_transformed, orientation_transformed

# Example usage:
# Sensor data in NWU frame
position_nwu = np.array([10, -5, 2])  # [North, West, Up] in meters
velocity_nwu = np.array([2, -1, 0.5])  # [North, West, Up] in m/s

# Orientation as quaternion (w, x, y, z) in NWU frame
orientation_nwu = np.array([1, 0, 0, 0])  # Identity quaternion (no rotation)

# Sensor mounting orientation (Euler angles in FRD frame)
# Example: Roll = 10°, Pitch = 15°, Yaw = 20°
roll = 10  # in degrees
pitch = 15  # in degrees
yaw = 20  # in degrees

# Step 1: Convert the mounting orientation (Euler angles) to quaternion
mounting_quaternion = euler_to_quaternion(roll, pitch, yaw)

# Step 2: Convert from NWU to NED frame
position_ned, velocity_ned, orientation_ned = nwu_to_ned(position_nwu, velocity_nwu, orientation_nwu)

# Step 3: Apply the sensor's mounting orientation (quaternion) to the NED data
position_final, velocity_final, orientation_final = apply_sensor_orientation(position_ned, velocity_ned, orientation_ned, mounting_quaternion)

# Output the transformed data
print("Position in NED (after applying mounting orientation):", position_final)
print("Velocity in NED (after applying mounting orientation):", velocity_final)
print("Orientation in NED (after applying mounting orientation):", orientation_final)
