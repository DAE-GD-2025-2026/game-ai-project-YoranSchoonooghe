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
