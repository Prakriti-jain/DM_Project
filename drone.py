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

    return None, None, b


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
    delivery_destinations = [(10, 10), (2, 6), (10, 2), (1, 10)]
    start = (0, 0)
    height_standard = 6
    total_drones = int(input("Enter the number of drones (n): "))

    unassigned_destinations = set(delivery_destinations)
    assigned_destinations = set()

    all_paths = [[] for _ in range(total_drones)]
    all_costs = [[] for _ in range(total_drones)]
    current_locations = [start] * total_drones
    batteries = [30] * total_drones

    # ... (previous code)

    while unassigned_destinations:
        for drone_id in range(total_drones):
            current_destinations = find_unassigned_destination(unassigned_destinations, assigned_destinations)
            if current_destinations:
                current_destination = min(current_destinations,
                                          key=lambda dest: heuristic(current_locations[drone_id], dest))
                shortest_path, cost, battery = astar_search(building_map, current_locations[drone_id],
                                                            current_destination, height_standard, batteries[drone_id],
                                                            charging_points)
                if shortest_path:
                    all_paths[drone_id].append(shortest_path)
                    all_costs[drone_id].append(cost)
                    current_locations[drone_id] = current_destination
                    batteries[drone_id] = battery
                    assigned_destinations.add(current_destination)
                    unassigned_destinations.remove(current_destination)
                else:
                    print(f"No valid path for Drone {drone_id + 1} to delivery destination: {current_destination}")

                # Check for charging after each delivery
                if current_destination in charging_points and batteries[drone_id] < 100:
                    batteries[drone_id] = min(batteries[drone_id] + 10, 100)
                    print(f"Drone {drone_id + 1} charged. Battery: {batteries[drone_id]}%")

    for drone_id in range(total_drones):
        for i in range(len(all_costs[drone_id])):
            all_costs[drone_id][i] = (all_costs[drone_id][i] * 30) / 60

    for drone_id in range(total_drones):
        if all_paths[drone_id]:
            print(f"All Delivery Paths for Drone {drone_id + 1}:")
            for i, (path, cost) in enumerate(zip(all_paths[drone_id], all_costs[drone_id])):
                print(f"Delivery {i + 1}: {path} (Time: {cost} minutes)")
            print(f"Drone {drone_id + 1} Final Battery: {batteries[drone_id]}%")

            plot(all_paths[drone_id], building_map, all_costs[drone_id])
        else:
            print(f"No valid paths found for deliveries by Drone {drone_id + 1}")


if __name__ == "__main__":
    main()
