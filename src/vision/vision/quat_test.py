import numpy as np

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def conjugate(self):
        # Return the conjugate of the quaternion (w, -x, -y, -z)
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def __mul__(self, other):
        # Multiply this quaternion by another quaternion
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        return Quaternion(w, x, y, z)

    def __repr__(self):
        return f"({self.w}, {self.x}, {self.y}, {self.z})"

# Define the quaternion for a 90-degree rotation about the Z-axis
q_Z90 = Quaternion(np.sqrt(2)/2, 0, 0, np.sqrt(2)/2)

# Define the quaternion for a 180-degree rotation about the X-axis
q_X180 = Quaternion(0, 1, 0, 0)

# The combined frame rotation quaternion (ENU to NED)
q_frame_rotation = q_Z90 * q_X180

# Define the quaternion representing the orientation in the ENU frame (example)
q_ENU = Quaternion(0.7071068,0,0,0.7071068)

# Compute the NED quaternion using the formula q_NED = q_frame_rotation * q_ENU * q_frame_rotation.conjugate()
q_NED = q_frame_rotation * q_ENU * q_frame_rotation.conjugate()

# Print the resulting NED quaternion
print("q_NED:", q_NED)
