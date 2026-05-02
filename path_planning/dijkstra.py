import heapq

def dijkstra(grid_map):
    """
    Finds the shortest path from start to goal using Dijkstra's algorithm.
    Returns: (path, visited_order)
      - path: list of (row,col) from start to goal
      - visited_order: list showing which cells were explored (for animation)
    """
    start = grid_map.start
    goal  = grid_map.goal

    # Priority queue: (cost, row, col)
    # heapq always pops the lowest cost item first
    queue = [(0, start)]

    # Track where we came from — to reconstruct the path later
    came_from = {start: None}

    # Track the cost to reach each cell
    cost_so_far = {start: 0}

    visited_order = []  # for animation

    while queue:
        current_cost, current = heapq.heappop(queue)

        if current == goal:
            break  # found the goal!

        visited_order.append(current)

        for neighbor in grid_map.get_neighbors(*current):
            new_cost = cost_so_far[current] + 1  # each step costs 1

            # Only visit this neighbor if we found a cheaper way to reach it
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(queue, (new_cost, neighbor))

    # Reconstruct path by walking backwards from goal to start
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from.get(current)
    path.reverse()

    # If path doesn't start at start, no path was found
    if path[0] != start:
        return [], visited_order

    return path, visited_order