# Game AI Project
This project contains the exercises and assignments for Algorithms 2 (GameAI).

## Assignment 1: Flocking + Spatial Partitioning
### Flock
The first assignment is inspired by Craig Reynolds' Boids algorithm. Each agent in the flock follows 3 steering behaviors:

- Separation: Avoid neighbors
- Cohesion: Steer towards neighbors
- Alignment: Match neighbors' velocity

These 3 steering behaviors are combined in a blended steering behavior. Additionally, there are 2 more behaviors to the flock:

- Seek: Move towards a target
- Wander: Move to a random position on a circle in front of the agent

These 5 behaviors shape the movement of the flock.

### Priority Steering
There is another agent in the scene, an agent that every agent in the flock tries to evade. For this, another steering behavior is needed:

- Evade: Move away from the predicted position of the moving target

<p align="center">
  <img src="Resources/FlockEvading.gif" width="400"/>
</p>

This behavior should overrule any of the other behaviors, and therefore it is not added to the blended steering. Instead, the flock has a priority steering with the following behaviors:

1) Evade
2) Blended Steering of the previous 5 behaviors

### Spatial Partitioning
Every update, each agent checks if every other agent in the flock is close enough to become one of its neighbors. This is a rather expensive task, with a complexity of $$O(n^2)$$. Spatial Partitioning improves this operation by dividing the world space in a number of cells. Instead of checking all agents, each agent checks which cells in the cell space overlap with the agents' neighborhood. For every cell that overlaps, the agent checks which agents in the cell are close enough to become a neighbor.

<p align="center">
  <img src="Resources/SpacePartitioning.gif" width="400"/>
</p>

## Assignment 2: A* Pathfinding + Navigation Meshes
### Graphs
Graphs are datastructures made up of nodes and connections. In pathfinding and navigation, graphs can be used to make an agent move from one node to the other, by travelling along a connection between the nodes.

### A* Pathfinding
The A* pathfinding is an algorithm to find the shortest path between a starting point and a target point. A* algorithms are popular in games due to their efficiency, allowing fast computation and making them suitable for real-time applications. A* is also applicable for weighted graphs, and can for instance be used to find the shortest path from start to end for an agent, while taking into account the terrain obstacles. Blue tiles represent water, which block the path, and orange tiles represent mud, which has a larger weight because it is harder to cross.

### Navigation Meshes
Instead of using a waypoint graph, which allows navigation from node to node using connections, a navigation mesh can be used. A navigation mesh allows an agent to move to any point within a walkable space, defined by a navigation polygon. In the images below, the middle image shows a navigation polygon (yellow) for the level in the left image. The navigation polygon is made up of triangles, where the edges will be used to generate nodes for a navigation graph. There are multiple ways to create this navigation graph, for instance by creating a node at the center of each edge which is shared by 2 triangles. The connections of the graph can then be defined by connecting all nodes in a triangle, leading to 1 connection for a triangle with 2 nodes and 3 connections for a triangle with 3 nodes. The right image below shows the resulting navigation graph for the navigation polygon, where the red circles visualize the nodes and the blue lines represent the connections.

An agent can now move from a starting point to a target point, by adding the start and end nodes to the navigation graph, and by running the A* algorithm on the graph.

### Path Smoothing
The previous path is obviously not the optimal solution. There are several path smoothing techniques to get a shorter path, for instance by using the Simple Stupid Funnel Algorithm ([SSFA](http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html)). SSFA treats the shared edges of the triangles that the path crosses as portals. For each portal, P1 and P2 are the right and left points of the portal, respectively, with respect to the start point.

The SSFA algorithm walks down while evaluating the right and left points of the portal using legs, which creates a funnel. If the legs cross, a corner point is added and the loop restarts. The result is a more natural and straight path instead of a zigzagging path. Below, the cyan path represents the optimized path using SSFA.
