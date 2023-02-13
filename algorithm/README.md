# The Algorithm

This algorithm is inspired by [this article](https://www.scitepress.org/papers/2007/20528/20528.pdf) and [this software](https://v22.wiki.optitrack.com/index.php?title=Rigid_Body_Tracking). It absorbed some elements and ideas of these proposed systems.

## Description

### Input

The algorithm will receive a point cloud for analysis and a list of rigid bodies to find within it.

The point cloud is described as a set of $p$ points scattered in space. A list of $n$ rigid bodies is given, each rigid body $r$ is a set of $r_i \ (1 \leq i \leq n)$ point coordinates. Notice how
$$\sum _{i=1}^{n}r_i = p$$ 
must be true, otherwise it would imply in occlusions in the point cloud or unknown rigid bodies in the scene.

### Output

The output must be a partition of the initial point cloud: a list of $n$ set of points, each set containing the points judged to be part of a separate rigid body. Knowing that, rigid bodies *must not* share any number of points: all rigid bodies should have their unique set of points in space.

## Current Version 

Currently, the partition algorithm (in the function `partition`) can be described as:

1. The function is called with the following parameters:
   1. Scene point cloud $p_c$ composed of $p$ points
   2. A list containing $n_{rb}$ RigidBody objects present in the scene
   3. The double of the maximum capturing error $2 \cdot \epsilon$
2. The distance matrix of the point cloud received is calculated
3. For each valid interval $v_i$ in an interval set generated by the internal distances, search in $p_c$ distance matrix if there is any similar distance $d_{ij}$ 
- If $d_{ij}$ is inside any $v_i$ of the interval set, $d_{ij}$ is considered similar
- For each similar $d_{ij}$, vertices $i$ and $j$ will be linked in a graph $G_d$
4. The Girvan-Newman community detection algorithm will be used to find separate the rigid bodies by cutting the edges with the highest betweeness centrality. 
Let $e_i$ being the ideal number of distances that should be detected and $e_d$ the actual number of distances detected, the algorithm will run $e_d - e_i$ times (thus cutting $e_d - e_i$ edges)
5. Having the excess edges removed of the graph, it should find $n_{rbd}$ disconnected subgraphs of the modified $G_d$
6. If the partition was done correctly, then $(n_{rb} = n_{rbd})$ is true, and it returns a list of sets containing point coordinates, each set representing a different rigid body.
   If not, it returns an empty list 

### Observations

General features:

- It is noticeable that the algorithm abstracts the matching coordinates problem to a graph problem.
- In some cases when two separate rigid bodies are close, a distance $d_ij$ between two of their points could coincide with some of their internal distances, connecting them in the graph abstraction. These coincidences will be called **ambiguities**.
- The kinematics module is made to simulate some point cloud capturing conditions.
- The partition module contains the actual algorithm.
- The tracking module has the definition of the RigidBody class.

Regarding the scene file:

- Every length unit is in centimeters.
- The main code simulates a point cloud of a captured scene composed of a certain quantity of the Parrot Mambo and the DJI Tello drones.
  - They are represented by a constellation of markers.
  - Each of them are randomly rotated and randomly translated into a predefined cube of 4 meters. 
- A point cloud is created by taking each point of the rigid bodies in the scene in a random sequence, and displacing them in a random direction by a maximum of $\epsilon$ centimeters, being $\epsilon$ the maximum capture error. 
- The partition algorithm is then called and timed. 
- Change the `save` parameter of the main function if saving the point cloud is convenient.

### Problems

- This version does not work so well with rigid bodies composed by less than 4 markers.
- The computation time is roughly proportional to the number of ambiguities detected (variable, depending on the scene).
 
---