# Importing modules...
import numpy as np
from scipy.spatial.transform import Rotation

# Random rotation matrix
def randR():
    return Rotation.from_euler('xyz', [np.random.randint(0, 360), np.random.randint(0, 360), np.random.randint(0, 360)], degrees=True).as_matrix()

# Random translation matrix
def randt(L): # Confined in a cube with an edge of length L
    return np.array([[2*L*np.random.random_sample()-L] for _ in range(3)])

# Rotate a point P in a observed frame O to be in respect to a main frame M 
def R(O, M):
    return np.array([[np.dot(O[0], M[0]), np.dot(O[1], M[0]), np.dot(O[2], M[0])],
                     [np.dot(O[0], M[1]), np.dot(O[1], M[1]), np.dot(O[2], M[1])],
                     [np.dot(O[0], M[2]), np.dot(O[1], M[2]), np.dot(O[2], M[2])]])

def RPY(R):
    return Rotation.from_rotvec(R).as_euler('xyz', degrees=True)

# Make coordinates homogeneous
def to_homo(points):
    return np.vstack((points, np.ones(points.shape[1]))) 

# Joins the rotation and translation matrices to make a homogeneous transformation
def join_homo(R, t):
    return np.vstack((np.hstack((R, t)), np.array([0, 0, 0, 1])))

# Extract the rotation and translation matrices from the homogeneous transformation
def split_homo(H):
    return H[0:3, 0:3], H[0:3 , [-1]]

# Returns the inverse of a homogeneous transformation matrix
def inverse_homo(H):
    R, t = split_homo(H)
    
    return join_homo(R.T, -R.T @ t)

# Returns the vertices of an unit cube with corner in origin
def cube(position=np.zeros(3).reshape(-1,1), size=1/2):
    vertices = np.array([[(V>>2)&1, (V>>1)&1, (V>>0)&1] for V in range(8)]).T*size+position

    return vertices

# Returns the vertices of an unit square with corner in origin
def square(position=np.zeros(3).reshape(-1,1), size=1/2):
    vertices = np.array([[(V>>1)&1, (V>>0)&1, 0] for V in range(4)]).T*size+position

    return vertices