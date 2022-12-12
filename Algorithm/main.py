import numpy as np
import scipy as sp
import time as tm

# Homebrewed modules
from partition import *
from kinematics import *

def main(save):
    np.set_printoptions(precision=2)

    epsilon = 0.7 # Max capture error in centimeters
    n_rb = 9 # Number of rigid bodies in the scene
    # Parrot Mambo marker base (in centimeters)
    mambo = np.array([[-16, 16, 0], [16, 16, 0], [16, -16, 0], [-16, -16, 0], [0, 8, 0]])

    # Rotate and translate randomly all the Mambos in the scene
    scene_rb = np.array([random_translate(random_rotate(mambo, 30, 30, 180), 200, 200, 400) for _ in range(n_rb)])

    # Simulate random deviations in measurements of point coordinates
    point_cloud = random_displace(np.random.permutation(np.concatenate(scene_rb)), epsilon)

    # Calculate internal distances of the known rigid bodies
    scene_rb_distances = np.concatenate([sp.spatial.distance.pdist(rb, metric='euclidean') for rb in scene_rb])

    print('\n⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯')
    print('\nRigid bodies in scene:\n')

    # Initiate timer
    start_ms = tm.time_ns() / 1e6
    # Identify rigid bodies, save which markers define a rigid body
    # The tolerance must be at least twice the maximum system error 
    # This represents the worst case scenario for distance difference between 2 points
    all_rb = rb_identify(point_cloud, n_rb, scene_rb_distances, epsilon * 2)
    # End timer
    end_ms = tm.time_ns() / 1e6

    if all_rb.any():
        print('Number of rigid bodies found:', len(all_rb))
    else:
        print('No correct match!')

    print('\nTime elapsed: {:.5f} ms'.format((end_ms-start_ms)))
    print('\n⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯\n')

    # Saving point cloud routine
    if save:
        with open("PCD.xyz", 'w') as xyz_file:
            for rb in all_rb:
                for c in rb:
                    xyz_file.write(f"{c[0]:11.6f} {c[1]:11.6f} {c[2]:11.6f}\n")
        
        print('Point cloud saved in PCD.xyz!\n')

if __name__ == '__main__':
    main(save=True)