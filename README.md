Maze Solver : Drone Path Planning

Introduction
The goal of this project is to efficiently guide drones through a maze-like area while taking into account practical limitations like battery capacity, charging points, and building heights. The A* search method, which is well-known for its effectiveness and optimality, is the main algorithm used for path planning.

Usage
1. Input Map: Provide a map file representing the maze-like environment, with charging points marked as 0.
2. Delivery Destinations: Define delivery destinations as a list of coordinates.
3. Number of Drones: Input the number of drones participating in the delivery system.
4. Run the Program: Execute the main() function. The program will output delivery paths, time estimates, and final battery levels for each drone.

Code Structure
1. load_map(filename): This function reads a file containing a maze-like map, where 0s represent charging points. It returns a 2D list (building_map) representing the maze and a set of coordinates (charging_points) where drones can recharge.

2. is_valid_coord(coord, height, width, height_standard, building_map): Checks whether a given coordinate is within the maze boundaries (height and width) and doesn't exceed a specified height standard. It also ensures the coordinate does not collide with obstacles in the building_map.

3. heuristic(current, goal): Computes a heuristic estimate for the cost of reaching the goal from the current position. In this case, it uses the Manhattan distance, which is the sum of the absolute differences in the x and y coordinates.

4. find_unassigned_destination(destinations, assigned_destinations): Determines the set of destinations that haven't been assigned to drones by subtracting the assigned_destinations from the complete set of destinations (destinations).

5. astar_search(building_map, start, goal, height_standard, battery, charging_points): Implements the A* search algorithm, considering factors such as dynamic battery management. It finds the shortest path from start to goal, calculates the associated cost, and updates the battery level. The function handles obstacles, charging points, and adjusts battery levels based on the path.

6. plot(all_paths, building_map, costs): Visualizes drone paths on the building_map using Matplotlib. Each drone's path is plotted with a unique color. This function provides a graphical representation of the drone movements.

7. main(): The main function orchestrates the entire drone path planning process. It loads the map, defines delivery destinations, specifies the number of drones, and executes the A* search for each drone. The final delivery paths, time estimates, and battery levels are displayed. The function also incorporates charging points to optimize battery levels during deliveries.
