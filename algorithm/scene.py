import numpy as np
import time as tm

# Custom modules
from tracking import *
from partition import *
from kinematics import *

def scene(save):
    np.set_printoptions(precision=2)

    capture_error = 0.7 # Max capture error in centimeters
    # Parrot Mambo marker base (in centimeters)
    n_mambo, mambo = 5, np.array([[-8, 8, 0], [8, 8, 0], [8, -8, 0], [-8, -8, 0], [0, 4, 0]])
    # DJI Tello marker base (in centimeters)
    n_tello, tello = 0, np.array([[-12, 12, 0], [12, 12, 0], [-12, -12, 0], [0, 0, 0]])
    
    # Rotate and translate randomly all the drones
    scene_mambos = [random_translate(random_rotate(mambo, 30, 30, 180), 200, 200, 400) for _ in range(n_mambo)]
    scene_tellos = [random_translate(random_rotate(tello, 30, 30, 180), 200, 200, 400) for _ in range(n_tello)]
    
    scene_rb = scene_mambos + scene_tellos
    
    # Generate point cloud by simulating random deviations in measurements of point coordinates
    scene_pcd = random_displace(np.random.permutation(np.concatenate(scene_rb)), capture_error)
    
    # Generate a list of Rigid Bodies in the scene
    scene_rb = list(map(RigidBody, scene_rb))

    print('\n⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯')
    print('# SCENE ANALYSIS\n')

    start_ms = tm.time_ns() / 1e6 # Initiate timer
    # Identify rigid bodies, save which markers define a rigid body
    all_rb, ambiguities = partition(scene_pcd, scene_rb, capture_error * 2)
    # The tolerance must be at least twice the maximum system error 
    # This represents the worst case scenario for distance difference between 2 points
    end_ms = tm.time_ns() / 1e6 # End timer

    if all_rb: # Not empty
        print(f'  > {len(all_rb)} rigid bodies found\n')
    else: # Empty
        print('>  No correct match!\n')
        
    print(f'  > {ambiguities} ambiguities detected\n')

    print(f'Time elapsed: {end_ms-start_ms:.2f} ms')
    
    # Collecting scene point cloud for analysis
    if save:
        with open("Collect/PCD.xyz", 'w') as xyz_file:
            for c in scene_pcd:
                xyz_file.write(f"{c[0]:11.6f} {c[1]:11.6f} {c[2]:11.6f}\n")
        
        print('\nPoint cloud saved in PCD.xyz')
    print('⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯\n')

if __name__ == '__main__':
    scene(save=False)