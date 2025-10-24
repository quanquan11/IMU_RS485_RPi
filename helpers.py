import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.spatial.transform import Rotation as R

from environments import ERR_COVAR_MATRIX, MEASURE_NOISE_MATRIX, PROC_NOISE_COVAR_MATRIX, SEGMENT_LENGTH_CM

def create_kalman():
    """Setting up Kalman filter, define it's parameters."""

    kf = KalmanFilter(dim_x = 4, dim_z = 4)

    kf.F = np.eye(4)
    kf.H = np.eye(4)
    kf.P *= ERR_COVAR_MATRIX
    kf.R *= MEASURE_NOISE_MATRIX
    kf.Q *= PROC_NOISE_COVAR_MATRIX

    return kf

def apply_kalman_filter(kf, quaternion):
    """Apply Kalman filter to quaternion readings."""

    kf.predict()
    kf.update(quaternion)

    return kf.x

def quaternion_to_roll(quaternion):
    """Convert quaternion to pitch angle (theta in degrees)."""

    # Flatten to 1D array if needed
    quat = np.array(quaternion).flatten()

    # Convert from [w, x, y, z] to [x, y, z, w] for scipy
    quat_scipy = np.array([quat[1], quat[2], quat[3], quat[0]])

    r = R.from_quat(quat_scipy)
    euler = r.as_euler("xyz", degrees = True)

    return euler[0]

def calculate_displacement(roll_angle, node_id, inverse = False):
    """Calculate horizontal displacement using trigonometry."""

    # Validate node_id is within bounds
    if node_id < 0 or node_id >= len(SEGMENT_LENGTH_CM):
        raise ValueError(f"Invalid node_id {node_id}. Expected 0-{len(SEGMENT_LENGTH_CM)-1}. Check SEGMENT_LENGTH_CM configuration.")

    roll_rad = np.radians(roll_angle)

    if inverse == True:
        return -1 * SEGMENT_LENGTH_CM[node_id] * np.sin(roll_rad)
    else:
        return SEGMENT_LENGTH_CM[node_id] * np.sin(roll_rad)