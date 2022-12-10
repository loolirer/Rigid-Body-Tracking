import numpy as np
from scipy.spatial.transform import Rotation

# Returns a normalized a vector
def normalize(vector):
    norm = np.linalg.norm(vector)
    if norm == 0:
       return vector
    return vector / norm

# Displace each point of a point cloud a maximum of delta units in a random direction
def random_displace(point_cloud, delta):
    displaced_point_cloud = np.array(point_cloud, dtype=float)

    for point in displaced_point_cloud:
        random_direction = normalize(np.random.rand(len(point)) * 2 - 1) * (delta * np.random.rand())
        point += random_direction

    return displaced_point_cloud

# Randomly translates a rigid body model inside a parallelepiped centered in the origin
def random_translate(model, x_t, y_t, z_t):
    displacement = np.array([np.random.randint(-x_t, x_t), np.random.randint(-y_t, y_t),np.random.randint(z_t)])

    translated_model = np.copy(model)

    for marker in translated_model:
        marker += displacement

    return translated_model

# Randomly rotates a rigid body model in the axis x, y and z
def random_rotate(model, x_r, y_r, z_r):
    rotation_x = np.array(Rotation.from_euler('x',  np.random.randint(-x_r, x_r), degrees=True).as_matrix())
    rotation_y = np.array(Rotation.from_euler('y',  np.random.randint(-y_r, y_r), degrees=True).as_matrix())
    rotation_z = np.array(Rotation.from_euler('z',  np.random.randint(-z_r, z_r), degrees=True).as_matrix())
    
    rotated_model = np.array(np.copy(model), dtype=float)

    for marker in range(len(rotated_model)):
        rotated_model[marker] = rotation_x @ rotation_y @ rotation_z @ rotated_model[marker]

    return rotated_model