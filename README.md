# Rigid Body Tracking

This repository is dedicated to develop efficient real time tracking algorithms for known rigid bodies in a point cloud.

## Concepts 

Some initial concepts are necessary for the understanding of the algorithm:

- A **point cloud** is a discrete set of points in a cartesian space, either 2D or 3D.
It can be obtained in a multitude of ways, such as in simulations or marker based optical tracking systems.
- A **rigid body** is an user defined set of points that represents a body to be tracked.
These points are spacially stactic in relation to each other. That means their distances should remain mostly constant over time.
- The algorithm is made so it can *partition* the point cloud into subsets that represents the known rigid bodies.

## Goals and Features

Some objetives are considered in the development process:

- Since this is aimed for real time applications such as optical tracking systems, the process must be the most time efficient as possible.
- Achieving this in polynomial time is key. These problems can become exponencially bigger with ease.
- Having valid partitions in a reasonable time limit *most* of the time is chosen over having valid partitions *all* the time.

This project will stick to some features, initially:

- The algorithm will be done in Python3.
- Some external packages will be used to aid the computation process, such as:
  - NumPy.
  - SciPy.
  - NetworkX.  

## The Algorithm

### Input

The algorithm will receive a point cloud for analysis and a list of rigid bodies to find within it.

The point cloud is described as a set of $p$ points scattered in space. A list of $n$ rigid bodies is given, each rigid body $r$ is a set of $r_i \ (1 \leq i \leq n)$ point coordinates. Notice that $\sum _{i=1}^{n}r_i = p$ must be true, otherwise it would imply in occlusions in the point cloud or unknown rigid bodies in the scene.

### Output

The output must be a partition of the initial point cloud: a list of $n$ set of points, each set containing the points judged to be part of a separate rigid body. Note that different rigid bodies *must not* share any number of points: all rigid bodies should have their unique set of points in space.

---
