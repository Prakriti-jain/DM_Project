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

# Define the A* search algorithm.
def astar_search(building_map, start, goal, height_standard):
    height = len(building_map)
    width = len(building_map[0])
    open_list = [(0, start)]
    came_from = {}
    g_score = {coord: float('inf') for coord in [(x, y) for x in range(height) for y in range(width)]}
    g_score[start] = 0

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
                    tentative_g_score = g_score[current] + 1

                    if tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score, neighbor))

    return None  # No path found

# Main function
def main():
    building_map = load_map("test.txt")
    start = (0, 0)
    goal = (4, 5)  # Adjusted goal coordinates
    height_standard = 5  # Height standard for the drone

    shortest_path = astar_search(building_map, start, goal, height_standard)

    if shortest_path:
        print("Shortest Path Coordinates:")
        for coord in shortest_path:
            print(coord)
    else:
        print("No valid path found")

if __name__ == "__main__":
    main()