import time
from path_planning.map        import GridMap
from path_planning.dijkstra   import dijkstra
from path_planning.astar      import astar
from path_planning.visualizer import animate_search, plot_comparison
from path_planning.simulation import run_simulation

def main():
    print("=" * 50)
    print("  Mobile Robot Path Planner")
    print("  Dijkstra vs A* Comparison")
    print("=" * 50)

    # --- Build the map ---
    grid = GridMap(rows=20, cols=20).build_default_map()
    print(f"\nMap: {grid.rows}x{grid.cols} grid")
    print(f"Start: {grid.start}  →  Goal: {grid.goal}")

    # --- Run Dijkstra ---
    print("\n[1/4] Running Dijkstra...")
    t0 = time.time()
    d_path, d_visited = dijkstra(grid)
    d_time = time.time() - t0
    print(f"  Path length:    {len(d_path)} steps")
    print(f"  Cells explored: {len(d_visited)}")
    print(f"  Time:           {d_time*1000:.2f} ms")

    # --- Run A* ---
    print("\n[2/4] Running A*...")
    t0 = time.time()
    a_path, a_visited = astar(grid)
    a_time = time.time() - t0
    print(f"  Path length:    {len(a_path)} steps")
    print(f"  Cells explored: {len(a_visited)}")
    print(f"  Time:           {a_time*1000:.2f} ms")

    # --- Animate Dijkstra ---
    print("\n[3/4] Animating Dijkstra search...")
    animate_search(grid, d_visited, d_path,
                   title="Dijkstra — Explores everything equally",
                   save_path="results/dijkstra.gif")

    # --- Animate A* ---
    print("\n      Animating A* search...")
    animate_search(grid, a_visited, a_path,
                   title="A* — Guided by heuristic toward goal",
                   save_path="results/astar.gif")

    # --- Side-by-side comparison ---
    print("\n      Saving comparison chart...")
    results = [
        {'name': 'Dijkstra', 'path': d_path,
         'visited': d_visited, 'time': d_time},
        {'name': 'A*',       'path': a_path,
         'visited': a_visited, 'time': a_time},
    ]
    plot_comparison(grid, results)

    # --- MuJoCo simulation ---
    print("\n[4/4] Running MuJoCo simulation with A* path...")
    run_simulation(a_path, grid,
                   title="MuJoCo — Robot following A* path")

    print("\n✓ All done! Check the results/ folder for saved outputs.")

if __name__ == "__main__":
    main()