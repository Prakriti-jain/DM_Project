import heapq

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

# Define the A* search algorithm with battery constraint.
def astar_search(building_map, start, goal, height_standard, battery_capacity):
    height = len(building_map)
    width = len(building_map[0])
    open_list = [(0, start)]
    came_from = {}
    g_score = {coord: float('inf') for coord in [(x, y) for x in range(height) for y in range(width)]}
    g_score[start] = 0
    battery = {coord: float('inf') for coord in [(x, y) for x in range(height) for y in range(width)]}
    battery[start] = battery_capacity  # Initialize battery at start position

    # Set the height of the goal building to -1 in the array during runtime
    building_map[goal[0]][goal[1]] = -1

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = [goal]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                neighbor = (current[0] + dx, current[1] + dy)

                if is_valid_coord(neighbor, height, width, height_standard, building_map):
                    movement_cost = 1
                    tentative_g_score = g_score[current] + movement_cost
                    tentative_battery = battery[current] - movement_cost

                    if tentative_g_score < g_score[neighbor] and tentative_battery >= 0:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        battery[neighbor] = tentative_battery
                        f_score = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score, neighbor))

    return None  # No path found

# Main function
def main():
    building_map = load_map("test.txt")
    start = (0, 0)
    goal = (4, 5)  # Adjusted goal coordinates
    height_standard = 5  # Height standard for the drone
    battery_capacity = 4  # Battery capacity for the drone

    shortest_path = astar_search(start, goal, height_standard, battery_capacity)

    if shortest_path:
        print("Shortest Path Coordinates:")
        remaining_battery = battery_capacity  # Initialize remaining battery
        for coord in shortest_path:
            print("Coordinate:", coord, "Remaining Battery:", remaining_battery)
            # Update the remaining battery based on the movement cost (assuming 1 unit of battery per step)
            remaining_battery -= 1
            if remaining_battery < 0 and coord != goal:
                print("Battery not sufficient")
                break
        if remaining_battery > 0:
            print("Final Remaining Battery:", remaining_battery)
    else:
        print("No valid path found")

if __name__ == "__main__":
    main()
