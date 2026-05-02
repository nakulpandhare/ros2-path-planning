import heapq

def heuristic(cell, goal):
    """
    Manhattan distance — straight-line step count ignoring obstacles.
    This guides A* toward the goal efficiently.
    """
    return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

def astar(grid_map):
    """
    Finds the shortest path using A* algorithm.
    Faster than Dijkstra because it uses a heuristic to guide the search.
    Returns: (path, visited_order)
    """
    start = grid_map.start
    goal  = grid_map.goal

    # Priority queue: (f_score, row, col)
    # f_score = g_score (actual cost) + h_score (estimated cost to goal)
    queue = [(0 + heuristic(start, goal), start)]

    came_from   = {start: None}
    g_score     = {start: 0}      # actual cost from start
    visited_order = []

    while queue:
        _, current = heapq.heappop(queue)

        if current == goal:
            break

        visited_order.append(current)

        for neighbor in grid_map.get_neighbors(*current):
            new_g = g_score[current] + 1

            if neighbor not in g_score or new_g < g_score[neighbor]:
                g_score[neighbor] = new_g
                f_score = new_g + heuristic(neighbor, goal)
                came_from[neighbor] = current
                heapq.heappush(queue, (f_score, neighbor))

    # Reconstruct path
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from.get(current)
    path.reverse()

    if not path or path[0] != start:
        return [], visited_order

    return path, visited_order