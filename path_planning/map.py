import numpy as np

class GridMap:
    """
    Represents the robot's world as a 2D grid.
    0 = free space, 1 = obstacle
    """

    def __init__(self, rows=20, cols=20):
        self.rows = rows
        self.cols = cols
        self.grid = np.zeros((rows, cols), dtype=int)  # start all free
        self.start = (1, 1)          # (row, col)
        self.goal  = (rows-2, cols-2)

    def add_obstacle(self, row, col):
        self.grid[row][col] = 1

    def add_obstacle_block(self, row, col, height, width):
        """Add a rectangular block of obstacles"""
        for r in range(row, row + height):
            for c in range(col, col + width):
                if self.is_valid(r, c):
                    self.grid[r][c] = 1

    def is_valid(self, row, col):
        """Check if a cell is inside the grid and not an obstacle"""
        return (0 <= row < self.rows and
                0 <= col < self.cols and
                self.grid[row][col] == 0)

    def get_neighbors(self, row, col):
        """
        Return all valid neighbors of a cell.
        We allow 4-directional movement (up, down, left, right).
        """
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for dr, dc in directions:
            nr, nc = row + dr, col + dc
            if self.is_valid(nr, nc):
                neighbors.append((nr, nc))
        return neighbors

    def build_default_map(self):
        """Create a map with some walls and obstacles for testing"""
        # Outer border
        for r in range(self.rows):
            self.grid[r][0] = 1
            self.grid[r][self.cols-1] = 1
        for c in range(self.cols):
            self.grid[0][c] = 1
            self.grid[self.rows-1][c] = 1

        # Some obstacle blocks (row, col, height, width)
        self.add_obstacle_block(3,  4,  8, 2)   # vertical wall
        self.add_obstacle_block(5, 10,  2, 6)   # horizontal wall
        self.add_obstacle_block(12, 6,  2, 8)   # another wall
        self.add_obstacle_block(3, 14,  6, 2)   # right side wall

        return self