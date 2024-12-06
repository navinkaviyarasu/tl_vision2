import numpy as np
from scipy.spatial.transform import Rotation as R

# Function to convert from ENU quaternion to NED quaternion
def enu_to_ned_quaternion(enu_quaternion):
    # Define the ENU to NED rotation matrix
    rotation_matrix = np.array([[0, 1, 0],
                                [1, 0, 0],
                                [0, 0, -1]])

    # Convert the rotation matrix to a quaternion (ENU to NED rotation)
    rotation = R.from_matrix(rotation_matrix)
    rotation_quaternion = rotation.as_quat()  # Quaternion representation of the rotation
    
    # Convert the ENU quaternion to a scipy Rotation object (assume ENU quaternion is [x, y, z, w])
    enu_rotation = R.from_quat(enu_quaternion)

    # Apply the rotation to the ENU quaternion using quaternion multiplication
    ned_rotation = rotation * enu_rotation

    # ned = R.from_euler('zyx', [90, 0, -180], degrees=True).apply(enu_quaternion)
    # print("NED", ned)

    # Return the resulting NED quaternion
    return ned_rotation.as_quat()

# Example: Quaternion in ENU frame (x, y, z, w)
enu_quaternion = np.array([0.5, -0.5, -0.5, 0.5])  # Example quaternion in ENU

# Convert to NED frame quaternion
ned_quaternion = enu_to_ned_quaternion(enu_quaternion)

# Print the result
print("ENU Quaternion:", enu_quaternion)
print("Converted NED Quaternion:", ned_quaternion)
