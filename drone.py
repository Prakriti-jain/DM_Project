import heapq
import matplotlib.pyplot as plt


# Define a function to read building heights from a .txt file and create a map.
def load_map(filename):
    building_map = []
    with open(filename, 'r') as file:
        for line in file:
            row = list(map(int, line.strip().split()))
            building_map.append(row)
    return building_map


# Define a function to check if a given coordinate is within the map boundaries and meets the height standard.
def is_valid_coord(coord, height, width, height_standard, building_map):
    x, y = coord
    return 0 <= x < height and 0 <= y < width and building_map[x][y] <= height_standard


# Define a function to calculate the heuristic (Manhattan distance) from a cell to the goal.
def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])


# Define the A* search algorithm.
def astar_search(building_map, start, goal, height_standard):
    height = len(building_map)
    width = len(building_map[0])
    open_list = [(0, start)]
    came_from = {}
    g_score = {coord: float('inf') for coord in [(x, y) for x in range(height) for y in range(width)]}
    g_score[start] = 0

    # Set the height of the goal building to -1 in the array during runtime
    building_map[start[0]][start[1]] = -1
    building_map[goal[0]][goal[1]] = -1

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = [goal]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, g_score[goal]

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                neighbor = (current[0] + dx, current[1] + dy)

                # Define cost for diagonal movements
                cost = 1 if dx == 0 or dy == 0 else 1.5

                if is_valid_coord(neighbor, height, width, height_standard, building_map):
                    tentative_g_score = g_score[current] + cost

                    if tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score, neighbor))

    return None, None  # No path found


def plot(all_paths, building_map, costs):
    # Copy the building map to avoid modifying the original
    path_map = [row.copy() for row in building_map]

    # Plot each path with a different color
    for i, (shortest_path, cost) in enumerate(zip(all_paths, costs)):
        # Mark the path on the map
        for coord in shortest_path:
            path_map[coord[0]][coord[1]] = -2

        # Plot the graph
        height = len(building_map)
        width = len(building_map[0])
        x = [coord[1] for coord in shortest_path]
        y = [width - coord[0] for coord in shortest_path]

        # Use a different color for each path
        color = plt.cm.viridis(i / len(all_paths))  # Choose a colormap (viridis in this case)
        plt.plot(x, y, label=f"Path {i + 1} (Time: {cost})", color=color)

    # Print the path map
    # for row in path_map:
    #     for value in row:
    #         if value == -1:
    #             print('G', end=' ')  # Goal
    #         elif value == -2:
    #             print('D', end=' ')  # Drone path
    #         else:
    #             print(value, end=' ')
    #     print()  # Move to the next row

    # Show legend
    plt.legend()

    # Set labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Drone paths')

    # Show the plot
    plt.show()


# Main function
def main():
    building_map = load_map("test.txt")
    delivery_destinations = [(5, 9), (6, 6), (9, 2)]
    start = (0, 0)
    height_standard = 6  # Height standard for the drone

    all_paths = []
    all_costs = []
    current_location = start

    for destination in delivery_destinations:
        shortest_path, cost = astar_search(building_map, current_location, destination, height_standard)
        if shortest_path:
            all_paths.append(shortest_path)
            all_costs.append(cost)
            current_location = destination
        else:
            print(f"No valid path to delivery destination: {destination}")
    for i in range(len(all_costs)):
        all_costs[i] = (all_costs[i] * 30)/60
    if all_paths:
        print("All Delivery Paths:")
        for i, (path, cost) in enumerate(zip(all_paths, all_costs)):
            print(f"Delivery {i + 1}: {path} (Time: {cost} minutes)")
        plot(all_paths, building_map, all_costs)
    else:
        print("No valid paths found for deliveries")


if __name__ == "__main__":
    main()
