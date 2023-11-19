import heapq
import matplotlib.pyplot as plt


def load_map(filename):
    building_map = []
    charging_points = set()

    with open(filename, 'r') as file:
        for row_idx, line in enumerate(file):
            row = list(map(int, line.strip().split()))
            for col_idx, value in enumerate(row):
                if value == 0:
                    charging_points.add((row_idx, col_idx))
            building_map.append(row)

    return building_map, charging_points


def is_valid_coord(coord, height, width, height_standard, building_map):
    x, y = coord
    return 0 <= x < height and 0 <= y < width and building_map[x][y] < height_standard


def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])


def find_unassigned_destination(destinations, assigned_destinations):
    unassigned_destinations = destinations - assigned_destinations
    return unassigned_destinations


def astar_search(building_map, start, goal, height_standard, battery, charging_points):
    b = battery
    height = len(building_map)
    width = len(building_map[0])
    open_list = [(0, start)]
    came_from = {}
    g_score = {coord: float('inf') for coord in [(x, y) for x in range(height) for y in range(width)]}
    g_score[start] = 0
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

            for point in path:
                if point in charging_points and b < 100:
                    if g_score[goal] > 20:
                        if b > 30:
                            continue
                        else:
                            b = min(b + 10, 100)
                            g_score[goal] += 2
                    else:
                        b = min(b + 10, 100)
                        g_score[goal] += 2

            b -= g_score[goal]
            return path, g_score[goal], b

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                neighbor = (current[0] + dx, current[1] + dy)
                cost = 1 if dx == 0 or dy == 0 else 1.5

                if is_valid_coord(neighbor, height, width, height_standard, building_map):
                    tentative_g_score = g_score[current] + cost

                    if neighbor in charging_points and b < 100:
                        b = min(b + 10, 100)

                    if tentative_g_score < g_score[neighbor] and b >= tentative_g_score:
                        g_score[neighbor] = tentative_g_score
                        came_from[neighbor] = current
                        f_score = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score, neighbor))

    return None, None, 0.0


def plot(all_paths, building_map, costs):
    path_map = [row.copy() for row in building_map]

    for i, (shortest_path, cost) in enumerate(zip(all_paths, costs)):
        for coord in shortest_path:
            path_map[coord[0]][coord[1]] = -2

        height = len(building_map)
        width = len(building_map[0])
        x = [coord[1] for coord in shortest_path]
        y = [height - coord[0] for coord in shortest_path]

        color = plt.cm.viridis(i / len(all_paths))
        plt.plot(x, y, label=f"Path {i + 1} (Time: {cost})", color=color)

    plt.legend()
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Drone paths')
    plt.show()


def main():
    building_map, charging_points = load_map("test.txt")
    height = len(building_map)
    width = len(building_map[0])

    print(f"Building Map Dimensions: {width} by {height}")

    print("Enter delivery destinations in the form (e.g., 10 10). Type 'exit' to finish.")
    delivery_destinations = []
    while True:
        n = input().split()

        if n == ['exit']:
            break
        else:
            if int(n[0]) < height and int(n[1]) < width:
                delivery_destinations.append((int(n[0]), int(n[1])))
            else:
                print("Invalid coordinates")
    height_standard = int(input("Height Standard: "))
    total_drones = int(input("Enter the number of drones (n): "))

    # Get unique starting points for each drone
    drone_starting_points = []
    for i in range(total_drones):
        while True:
            try:
                n = input(f"Enter starting point for Drone {i + 1} in the form (e.g., 0 0): ").split()
                start_x, start_y = int(n[0]), int(n[1])
                if 0 <= start_x < width and 0 <= start_y < height:
                    drone_starting_points.append((start_x, start_y))
                    break
                else:
                    print("Invalid coordinates. Try again.")
            except (ValueError, IndexError):
                print("Invalid input. Please enter valid coordinates.")

    unassigned_destinations = list(delivery_destinations)
    assigned_destinations = set()

    all_paths = [[] for _ in range(total_drones)]
    all_costs = [[] for _ in range(total_drones)]
    battery = int(input("Initial charge of drones"))
    batteries = [battery] * total_drones
    completion_times = [0] * total_drones

    # Initialize current locations for each drone
    current_locations = drone_starting_points.copy()

    for delivery_num in range(len(delivery_destinations)):
        for drone_id in range(total_drones):

            # Find the drone with the earliest completion time
            next_drone_id = min(range(total_drones), key=lambda i: completion_times[i])

            # Assign one destination to the chosen drone
            if unassigned_destinations:
                current_destination = unassigned_destinations.pop(0)
                shortest_path, cost, battery = astar_search(building_map, current_locations[next_drone_id],
                                                            current_destination, height, batteries[next_drone_id],
                                                            charging_points)
                if shortest_path:
                    all_paths[next_drone_id].append(shortest_path)
                    all_costs[next_drone_id].append(cost)
                    current_locations[next_drone_id] = current_destination  # Update current location
                    batteries[next_drone_id] -= cost  # Deduct battery for the travel cost
                    assigned_destinations.add(current_destination)
                    completion_times[next_drone_id] += cost
                else:
                    print(f"No valid path for Drone {next_drone_id + 1} to delivery destination: {current_destination}")

                # Check for charging after each delivery
                if current_destination in charging_points and batteries[next_drone_id] < 100:
                    batteries[next_drone_id] = min(batteries[next_drone_id] + 10, 100)
                    print(f"Drone {next_drone_id + 1} charged. Battery: {batteries[next_drone_id]}%")

    # ...

    for drone_id in range(total_drones):
        for i in range(len(all_costs[drone_id])):
            all_costs[drone_id][i] = (all_costs[drone_id][i] * 30) / 60

        for i, (path, cost) in enumerate(zip(all_paths[drone_id], all_costs[drone_id])):
            print(f"Delivery {i + 1} by Drone {drone_id + 1}: {path} (Time: {cost} minutes)")
        print(f"Drone {drone_id + 1} Final Battery: {batteries[drone_id]}%")

        plot(all_paths[drone_id], building_map, all_costs[drone_id])

if __name__ == "__main__":
    main()
