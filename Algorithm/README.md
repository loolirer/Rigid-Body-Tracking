# The Algorithm

This algorithm is inspired in [this article](https://www.scitepress.org/papers/2007/20528/20528.pdf). It absorbed the ideas of generating a matrix containing distance between points and using the deviation parameters of the capturing system to make validate these values.

## Description

### Input

The algorithm will receive a point cloud for analysis and a list of rigid bodies to find within it.

The point cloud is described as a set of $p$ points scattered in space. A list of $n$ rigid bodies is given, each rigid body $r$ is a set of $r_i \ (1 \leq i \leq n)$ point coordinates. Notice that $\sum _{i=1}^{n}r_i = p$ must be true, otherwise it would imply in occlusions in the point cloud or unknown rigid bodies in the scene.

### Output

The output must be a partition of the initial point cloud: a list of $n$ set of points, each set containing the points judged to be part of a separate rigid body. Note that different rigid bodies *must not* share any number of points: all rigid bodies should have their unique set of points in space.

## Current Version (v1)

Currently, the partition algorithm (in the function `rb_identify`) can be described as:

1. The function is called with the following parameters:
   1. Scene point cloud $pc$
   2. Pre-calculated list of distance matrices of the known rigid bodies $rb_d$
   3. The double of the maximum capturing error $2 \cdot \epsilon$
2. The distance matrix of the point cloud received is calculated
3. For each distance $d$ in $rb_d$, search in $p_c$ distance matrix if there is any similar distance $d_ij$ 
- If $|d-d_ij| < 2 \cdot \epsilon$, $d_ij$ is considered similar
- For each similar $d_ij$, vertices $i$ and $j$ will be linked in a graph $G_d$
4. Find the disconnected subgraphs of $G_d$ (*dfs* approach).
5. If the number of disconnected subgraphs isn't equal to the number of rigid bodies, it will call the `separate` function.
6. God knows what this function does, but it solves some ambiguities.
7. Returns a list of sets containing point coordinates.

### Observations

General features:

- It is noticeable that the algorithm abstracts the matching coordinates problem to a graph problem.
- In some cases when two separate rigid bodies are close, a distance $d_ij$ between two of their points could coincide with some of their internal distances, connecting them in the graph abstraction. These coincidences will be called **ambiguities**.
- The kinematics section is made to simulate some point cloud capturing conditions.
- The graph section is a homebrewed implementation of graphs.

Regarding the main section:

- Every length unit is in centimeters.
- The main code simulates a point cloud of a captured scene composed of a certain quantity of the indoor Parrot Mambo Drone.
  - A mambo is modeled by 3 points that matches approximately it's dimensions.
  - Each of them are randomly translated into a predefined parallelepiped. 
- A point cloud is created by taking each point of the mambos in the scene (in a random sequence) and displacing them randomly in a random direction with a maximum distance of the maximum error in the system.
- The main algorithm is then called and timed. 

### Problems

- This version does not support *some* ambiguities: some ambiguities are taken away by the `separate` function.
- The runtime of the function is relatively slow for expectations. It does not comport a large number of points efficiently.

---