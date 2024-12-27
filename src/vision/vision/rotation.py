import numpy as np
from scipy.spatial.transform import Rotation as R
# from pytransform3d.rotations import plot_basis


# Function to convert from ENU quaternion to NED quaternion
def enu_to_ned_quaternion(enu_quaternion):
    # Define the ENU to NED rotation matrix
    rotation_matrix = np.array([[0, 1, 0],
                                [1, 0, 0],
                                [0, 0, -1]])

    # # Convert the rotation matrix to a quaternion (ENU to NED rotation)
    # rotation = R.from_matrix(rotation_matrix)
    # rotation_quaternion = rotation.as_quat()  # Quaternion representation of the rotation
    
    # # Convert the ENU quaternion to a scipy Rotation object (assume ENU quaternion is [x, y, z, w])
    # enu_rotation = R.from_quat(enu_quaternion)
    # # rc = R.from_quat(enu_quaternion, scalar_first=True)

    # # Apply the rotation to the ENU quaternion using quaternion multiplication
    # ned_rotation = rotation * enu_rotation

    # # ned = R.from_euler('zyx', [90, 0, -180], degrees=True).apply(enu_quaternion)
    # # print("NED", ned)

    # # Return the resulting NED quaternion
    # return ned_rotation.as_quat()
    
    #NOTE: Workaround solution
    qx, qy, qz, qw = enu_quaternion

    q_vec_enu = np.array([qx, qy, qz])
    q_vec_ned = np.dot(rotation_matrix, q_vec_enu)
    ned_quaternion = np.hstack([q_vec_ned, qw])
    return ned_quaternion



# Example: Quaternion in ENU frame (x, y, z, w)
enu_quaternion = np.array([0, 0, 0.7071068, 0.7071068])  # Example quaternion in ENU

# Convert to NED frame quaternion
ned_quaternion = enu_to_ned_quaternion(enu_quaternion)


# plot_basis()

# Print the result
print("ENU Quaternion:", enu_quaternion)
print("Converted NED Quaternion:", ned_quaternion)
