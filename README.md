# Project 3: Path Planning - Python (F23)

Template code for Project 3 (Path Planning) in Python. See project instructions for more details.

## Algorithm Implementation

This implementation uses the **Breadth-First Search (BFS)** algorithm for path planning. BFS guarantees finding the shortest path in terms of number of steps (when all steps have equal cost) and is particularly effective for grid-based navigation problems.

### Why BFS was chosen:
- **Guaranteed shortest path** (in grid steps) when all movements have equal cost
- **Complete algorithm** - will find a solution if one exists
- **Simple implementation** that's easy to debug and verify
- **Well-suited** for the uniform-cost grid environment

### Performance Characteristics:
- **Time Complexity**: O(V + E) where V is vertices and E is edges
- **Space Complexity**: O(V) for storing visited nodes and queue
- **Optimality**: Finds shortest path in terms of number of grid cells traversed

### Comparison with Other Algorithms:
While A* might be more efficient with a good heuristic, and DFS uses less memory, BFS provides a good balance of simplicity, completeness, and optimality for this path planning application.

## Testing Your Code on Example Maps

This repository includes a helper script to visualize plan outputs compatible with the navigation web app. A tutorial is available here. You can run this file on your laptop (as long as you have Python 3 installed). To do so, do:

```bash
python path_planner_cli.py -m [PATH/TO/MAP] --start [START_i START_j] --goal [GOAL_i GOAL_j]
```
The map can be any map in the data/ folder. You can also make your own or use maps from the robot to test! The start and goal should be represented by cell indices. You can also upload maps to the web app so that you can choose good start and goal cells to test. To test a different algorithm, you can pass --algo [astar | bfs | dfs]. The default is bfs.

This script will generate the file out.planner, which you can upload to the web app.

Use python path_planner_cli.py -h to show usage.

# Run BFS on a map (BFS is default)
python path_planner_cli.py -m data/hospital_section.map --start 5 5 --goal 30 25

# Explicitly specify BFS algorithm
python path_planner_cli.py -m data/example.map --start 10 10 --goal 50 50 --algo bfs

# Try different algorithms for comparison
python path_planner_cli.py -m data/maze.map --start 2 2 --goal 30 30 --algo astar
python path_planner_cli.py -m data/emptymap8x8.map --start 1 1 --goal 6 6 --algo dfs

## Planning on the Robot

To run your code on the robot, you will first need to have made a map and be localized in it. Then, on the robot, do:
python robot_plan_path.py --goal [GOAL_x GOAL_y]

The goal is an x and y position in meters. Use the robot's web app to pick a good goal by clicking the cell you want to navigate to and noting its coordinates.

You will likely want to change the default collision radius. You can do this with argument -r [RADIUS].

Use python robot_plan_path.py -h to show usage.

## Simulation Video Demonstration

A video recording of the BFS path planning algorithm in action is available for viewing:

https://drive.google.com/file/d/1k1PrFOr62v12dNZjOoYTIMpL1qkBLuBA/view?usp=sharing
