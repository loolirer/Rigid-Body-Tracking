import numpy as np
import scipy as sp
import time as tm
import networkx as nx

# TRACKING

def rb_identify(point_cloud, rb_known_d, epslon):
    n_points = point_cloud.shape[0]
    n_ideal_edges = len(rb_known_d)

    point_cloud_d = sp.spatial.distance.pdist(point_cloud, metric='euclidean')

    d_graph = nx.Graph()

    # Graph generation
    for d in rb_known_d:
        for i in range(n_points):
            for j in range(i+1, n_points):
                if abs(d - point_cloud_d[(n_points - 1 + n_points - i)*(i)//2 - i + j - 1]) < epslon:
                    d_graph.add_edge(i, j)

    # Girvan-Newman of community detection
    for _ in range(d_graph.number_of_edges() - n_ideal_edges):
        edge_betweenness = nx.edge_betweenness_centrality(d_graph).items()
        edges_to_delete = sorted(edge_betweenness, key=lambda pair: -pair[1])[0][0]
    
        d_graph.remove_edge(*edges_to_delete)

    all_rb = [list(rb_points[1]) for rb_points in enumerate(nx.connected_components(d_graph))]

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

def r_displace(point_cloud, delta):
    point_cloud = np.array(point_cloud, dtype=float)
    for marker in point_cloud:
        rdir = normalize(np.random.rand(len(marker)) * 2 - 1) * delta * np.random.rand()
        for coord in range(len(marker)):
            marker[coord] += rdir[coord]
    return point_cloud

def r_translate(model, min, max):
    displacement = [np.random.randint(min,max), np.random.randint(min,max),np.random.randint(max)]

    d_model = np.copy(model)

    for marker in d_model:
        for coord in range(len(marker)):
            marker[coord] += displacement[coord]

    return d_model

# MAIN CODE

np.set_printoptions(precision=2)

max_error = 0.7 # in centimeters
n_rb = 15 # number of rigid bodies in scene
# mambo parrot marker base (in centimeters)
mambo = np.array([[-16, 16, 0], [16, 16, 0], [16, -16, 0]])

# translate randomly all the mambos in the scene
my_rb = [r_translate(mambo, -400, 400) for _ in range(n_rb)]

# simulate random deviations in measurements of point coordinates
all_points = r_displace(np.random.permutation(np.concatenate(my_rb)), max_error)

# calculate the distance matrices of the known rigid bodies
my_rb_d = np.concatenate([sp.spatial.distance.pdist(rb, metric='euclidean') for rb in my_rb])

print('\n---------------------------------------')
print('\nRigid bodies in scene:\n')

# initiate timer
start = tm.time_ns() / 1e6
# identify rigid bodies, save which markers define a rigid body
all_rb = rb_identify(all_points, my_rb_d, max_error * 2)
# end timer
end = tm.time_ns() / 1e6

if all_rb.any():
    print('Number of rigid bodies found:', len(all_rb))
else:
    print('No correct match!')

print('\nTime elapsed: {:.5f} ms'.format((end-start)))
print('\n---------------------------------------\n')