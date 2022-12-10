import numpy as np
import networkx as nx
from scipy.spatial.distance import pdist

# OPTIMIZATION

def float_set(float_list, delta):
    float_list, fset = sorted(float_list), []
    lower, upper = 0, 0
    while upper < len(float_list):
        if abs(float_list[upper] - float_list[lower]) < delta:
            upper += 1
            if upper == len(float_list) - 1:
                fset.append((float_list[lower]-delta, float_list[upper]+delta))
        else:
            fset.append((float_list[lower]-delta, float_list[upper-1]+delta))
            lower = upper

    return np.array(fset) 

# TRACKING

def rb_identify(point_cloud, n_rb, rb_known_distances, delta):
    n_points = point_cloud.shape[0]
    n_ideal_edges = len(rb_known_distances)

    point_cloud_distances = pdist(point_cloud, metric='euclidean')
    rb_known_distances = float_set(rb_known_distances, delta)

    # Graph generation
    distance_graph = nx.Graph()

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

    # Get disconnected components of the distance graph after the edge cut
    all_rb = [list(rb_markers[1]) for rb_markers in enumerate(nx.connected_components(distance_graph))]

    if len(all_rb) == n_rb: 
        return point_cloud[all_rb]
    else: 
        return np.array([]) # Incorrect partition will return a empty list