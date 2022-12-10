import numpy as np
import scipy as sp
import time as tm

# GRAPHS

def add_edge(adj, u, v):
    adj[u].add(v)
    adj[v].add(u)

def remove_edge(adj, u, v):
    adj[u].remove(v)
    adj[v].remove(u)

def dfs(adj, vis, s):
    stack = [s]
    vertices = []

    while len(stack):
        s = stack[-1]
        stack.pop()
        if not vis[s]:
            vertices.append(s)
            vis[s] = True
        for v in adj[s]:
            if not vis[v]:
                stack.append(v)

    return vertices

def bfs(adj, vis, s):
    queue = [s]
    vis[s] = True
    vertices = []

    while queue:
        s = queue.pop(0)
        vertices.append(s)
        for v in adj[s]:
            if vis[v] == False:
                queue.append(v)
                vis[v] = True

    return vertices

def is_complete(adj):
    for v in adj:
        if len(v) != len(adj) - 1:
            return False
    return True

def is_sub_complete(adj, sub):
    sub_size = len(sub)
    for v in sub:
        valid_edges = 0
        for u in adj[v]:
            if u in sub:
                valid_edges += 1
        if valid_edges != sub_size-1:
            return False
    return True

def is_connected(adj):
    visited = [False for _ in range(len(adj))]
    dfs(adj, visited, 0)
    for v in visited:
        if v == False:
            return False

    return True

def get_disconnected(adj):
    visited = [False for _ in range(len(adj))]
    disconnected = []
    for v in range(len(adj)):
        if visited[v] == False:
            disconnected.append(dfs(adj, visited, v))

    return disconnected

def separate(adj):
    visited = [False for _ in range(len(adj))]
    comp_graphs = []

    for v in range(len(adj)):
        if visited[v] == False:
            u = next(iter(adj[v]))
            possible = {v, u, *adj[v].intersection(adj[u])}
            if is_sub_complete(adj, possible): 
                comp_graphs.append(possible)
                for p in possible:
                    visited[p] = True

    for init_graph in comp_graphs:
        comp_disc_graphs = [list(init_graph)]
        p_graph = init_graph
        for c_graph in comp_graphs:
                c_intersec = p_graph.intersection(c_graph)
                if c_intersec == set():
                    comp_disc_graphs.append(list(c_graph))
                    p_graph = p_graph.union(c_graph)
        if sum([len(g) for g in comp_disc_graphs]) == len(adj):
            return np.array(comp_disc_graphs)

    return np.array([])

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
    simp_rb_known_d = np.array([d for rb in rb_known_d for d in rb[np.nonzero(rb)]])

    d_graph = [set() for _ in range(n_points)]

    for d in simp_rb_known_d:
        for i in range(n_points):
            for j in range(i+1, n_points):
                if abs(d - point_cloud_d[i][j]) < epslon:
                    add_edge(d_graph, i, j)

    all_rb = separate(d_graph)

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
n_rb = 5 # number of rigid bodies in scene
# mambo parrot marker base (in centimeters)
mambo = np.array([[-16, 16, 0], [16, 16, 0], [16, -16, 0]])

# translate randomly all the mambos in the scene
my_rb = [r_translate(mambo, -400, 400) for _ in range(n_rb)]

# simulate random deviations in measurements of point coordinates
all_points = r_displace(np.random.permutation(np.concatenate(my_rb)), max_error)

# calculate the distance matrices of the known rigid bodies
my_rb_d = np.array([dist_matrix(rb) for rb in my_rb], dtype=object)

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