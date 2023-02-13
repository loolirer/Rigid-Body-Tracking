from scipy.spatial.distance import pdist, squareform

# Class that represents a Rigid Body
class RigidBody:
    def __init__(self, markers):
        self.model = markers
        self.markers = self.model
        self.n_markers = len(self.model)
        self.d_list = pdist(self.model, metric='euclidean')
        self.d_matrix = squareform(self.d_list)
    
    def distance_matrix(self):
        return self.d_matrix

    def distance_list(self):
        return self.d_list

    def calculate_centroid(self):
        return [sum(self.model[:, axis])/self.n_markers for axis in range(3)]

    def get_markers(self):
        return self.markers