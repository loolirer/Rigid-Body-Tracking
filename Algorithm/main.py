import numpy as np
import scipy as sp
import time as tm

# GRAPHS

def add_edge(graph, u, v):
    graph[u].add(v)
    graph[v].add(u)

def dfs(graph, visited, s):
    stack = [s]
    vertices = []

    while len(stack):
        s = stack[-1]
        stack.pop()
        if not visited[s]:
            vertices.append(s)
            visited[s] = True
        for node in graph[s]:
            if not visited[node]:
                stack.append(node)
    
    return vertices

def get_disconnected(graph):
    visited = [False for _ in range(len(graph))]
    disconnected = []
    for u in range(len(graph)):
        if visited[u] == False:
            disconnected.append(dfs(graph, visited, u))

    return disconnected

# TRACKING

def dist_matrix(point_cloud):
    n_points = point_cloud.shape[0]
    d_m = np.zeros(shape=(n_points,n_points))
    for i in range(n_points):
        for j in range(i+1, n_points):
            d_m[i][j] = sp.spatial.distance.euclidean(point_cloud[i], point_cloud[j])
    return d_m

def rb_identify(point_cloud, rb_known_d, epslon):
    point_cloud_d = dist_matrix(point_cloud)
    n_points, n_rb = point_cloud_d.shape[0], rb_known_d.shape[0]
    rb_known_d = np.array([d for rb in rb_known_d for d in rb[np.nonzero(rb)]])

    d_graph = [set() for _ in range(n_points)]

    for d in rb_known_d:
        for i in range(n_points):
            for j in range(i+1, n_points):
                if abs(d - point_cloud_d[i][j]) < epslon:
                    add_edge(d_graph, i, j)

    rb = get_disconnected(d_graph)

    if len(rb) == n_rb:
        return point_cloud[rb]
    else:
        print(f'{n_rb} known rigid bodies, but {len(rb)} were detected.')
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
n_rb = 5 # number of rigid bodies in scene
# mambo parrot marker base (in centimeters)
mambo = np.array([[-16, 16, 0], [16, 16, 0], [16, -16, 0]])

# translate randomly all the mambos in the scene
my_rb = [r_translate(mambo, -400, 400) for _ in range(n_rb)]

# simulate random deviations in measurements of point coordinates 
all_points = r_displace(np.random.permutation(np.concatenate(my_rb)), max_error)

# calculate the distance matrices of the known rigid bodies
d_my_rb = np.array([dist_matrix(rb) for rb in my_rb], dtype=object)
print('\n---------------------------------------')
print('\nRigid bodies in scene:\n')

# initiate timer
start = tm.time_ns() / 1e6
# identify rigid bodies, save which markers define a rigid body
all_rb = rb_identify(all_points, d_my_rb, max_error * 2)
# end timer
end = tm.time_ns() / 1e6

if all_rb.any():
    print('Number of rigid bodies found:', len(all_rb))
else:
    print('\nNo correct match!')

print('\nTime elapsed: {:.5f} ms'.format(end-start))
print('\n---------------------------------------\n')