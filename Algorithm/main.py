import numpy as np
import scipy as sp
import time as tm
import networkx as nx

# OPTIMIZATION

def float_set(float_list, epslon):
    float_list, fset = sorted(float_list), []
    lower, upper = 0, 0
    while upper < len(float_list):
        if abs(float_list[upper] - float_list[lower]) < epslon:
            upper += 1
            if upper == len(float_list) - 1:
                fset.append((float_list[lower]-epslon, float_list[upper]+epslon))
        else:
            fset.append((float_list[lower]-epslon, float_list[upper-1]+epslon))
            lower = upper

    return np.array(fset) 

# TRACKING

def rb_identify(point_cloud, rb_known_distances, epslon):
    n_points = point_cloud.shape[0]
    n_ideal_edges = len(rb_known_distances)

    point_cloud_distances = sp.spatial.distance.pdist(point_cloud, metric='euclidean')
    rb_known_distances = float_set(rb_known_distances, epslon)

    distance_graph = nx.Graph()

    # Graph generation
    for interval in rb_known_distances:
        for i in range(n_points):
            for j in range(i+1, n_points):
                d_ij =  point_cloud_distances[(n_points - 1 + n_points - i)*(i)//2 - i + j - 1]
                if d_ij > interval[0] and d_ij < interval[1]:
                    distance_graph.add_edge(i, j)

    # Girvan-Newman of community detection
    for _ in range(distance_graph.number_of_edges() - n_ideal_edges):
        edge_betweenness = nx.edge_betweenness_centrality(distance_graph).items()
        edges_to_delete = sorted(edge_betweenness, key=lambda pair: -pair[1])[0][0]
    
        distance_graph.remove_edge(*edges_to_delete)

    all_rb = [list(rb_markers[1]) for rb_markers in enumerate(nx.connected_components(distance_graph))]

    if len(all_rb) == n_rb:
        return point_cloud[all_rb]
    else:
        return np.array([])

# KINEMATICS

def normalize(vector):
    norm = np.linalg.norm(vector)
    if norm == 0:
       return vector
    return vector / norm

def random_displace(point_cloud, delta):
    point_cloud = np.array(point_cloud, dtype=float)
    for marker in point_cloud:
        random_direction = normalize(np.random.rand(len(marker)) * 2 - 1) * delta * np.random.rand()
        for coord in range(len(marker)):
            marker[coord] += random_direction[coord]
    return point_cloud

def random_translate(model, min, max):
    displacement = [np.random.randint(min,max), np.random.randint(min,max),np.random.randint(max)]

    translated_model = np.copy(model)

    for marker in translated_model:
        for coord in range(len(marker)):
            marker[coord] += displacement[coord]

    return translated_model

def random_rotate(model):
    rotation_x = np.array(sp.spatial.transform.Rotation.from_euler('x',  np.random.randint(-30,30), degrees=True).as_matrix())
    rotation_y = np.array(sp.spatial.transform.Rotation.from_euler('y',  np.random.randint(-30,30), degrees=True).as_matrix())
    rotation_z = np.array(sp.spatial.transform.Rotation.from_euler('z',  np.random.randint(0,360), degrees=True).as_matrix())
    
    rotated_model = np.array(np.copy(model), dtype=float)

    for marker in range(len(rotated_model)):
        z_rotated = np.matmul(rotation_z, rotated_model[marker])
        yz_rotated = np.matmul(rotation_y, z_rotated)
        xyz_rotated = np.matmul(rotation_x, yz_rotated)
        rotated_model[marker] = xyz_rotated

    return rotated_model

# MAIN CODE

np.set_printoptions(precision=2)

max_error_cm = 0.7 # in centimeters
n_rb = 15 # number of rigid bodies in scene
# mambo parrot marker base (in centimeters)
mambo = np.array([[-16, 16, 0], [16, 16, 0], [16, -16, 0]])

# translate randomly all the mambos in the scene
my_rb = [random_translate(random_rotate(mambo), -400, 400) for _ in range(n_rb)]

# simulate random deviations in measurements of point coordinates
point_cloud = random_displace(np.random.permutation(np.concatenate(my_rb)), max_error_cm)

# calculate the distance matrices of the known rigid bodies
my_rb_distances = np.concatenate([sp.spatial.distance.pdist(rb, metric='euclidean') for rb in my_rb])

print('\n---------------------------------------')
print('\nRigid bodies in scene:\n')

# initiate timer
start_ms = tm.time_ns() / 1e6
# identify rigid bodies, save which markers define a rigid body
all_rb = rb_identify(point_cloud, my_rb_distances, max_error_cm * 2)
# end timer
end_ms = tm.time_ns() / 1e6

if all_rb.any():
    print('Number of rigid bodies found:', len(all_rb))
else:
    print('No correct match!')

print('\nTime elapsed: {:.5f} ms'.format((end_ms-start_ms)))
print('\n---------------------------------------\n')

'''if input('Save? [Y/N]: ') == 'Y':
    with open("Rigid Body Tracking/Algorithm/PCD.xyz", 'w') as xyz_file:
        for rb in all_rb:
            for c in rb:
                xyz_file.write(f"{c[0]:11.6f} {c[1]:11.6f} {c[2]:11.6f}\n")'''