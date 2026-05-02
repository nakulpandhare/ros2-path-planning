import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches

# Color scheme
COLORS = {
    'free':     [1.0,  1.0,  1.0],   # white
    'obstacle': [0.2,  0.2,  0.2],   # dark gray
    'start':    [0.0,  0.8,  0.0],   # green
    'goal':     [0.9,  0.1,  0.1],   # red
    'visited':  [0.6,  0.85, 1.0],   # light blue
    'path':     [1.0,  0.65, 0.0],   # orange
}

def build_image(grid_map, visited=[], path=[]):
    """Convert grid state into an RGB image array"""
    rows, cols = grid_map.rows, grid_map.cols
    img = np.ones((rows, cols, 3))

    # Draw base grid
    for r in range(rows):
        for c in range(cols):
            if grid_map.grid[r][c] == 1:
                img[r][c] = COLORS['obstacle']

    # Draw visited cells
    for (r, c) in visited:
        img[r][c] = COLORS['visited']

    # Draw final path on top
    for (r, c) in path:
        img[r][c] = COLORS['path']

    # Draw start and goal (always on top)
    sr, sc = grid_map.start
    gr, gc = grid_map.goal
    img[sr][sc] = COLORS['start']
    img[gr][gc] = COLORS['goal']

    return img

def animate_search(grid_map, visited_order, path, title, save_path=None):
    """
    Animate the search process step by step.
    Shows cells being explored, then reveals the final path.
    """
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.axis('off')

    # Legend
    legend = [
        mpatches.Patch(color=COLORS['start'],   label='Start'),
        mpatches.Patch(color=COLORS['goal'],    label='Goal'),
        mpatches.Patch(color=COLORS['visited'], label='Explored'),
        mpatches.Patch(color=COLORS['path'],    label='Final Path'),
        mpatches.Patch(color=COLORS['obstacle'],label='Obstacle'),
    ]
    ax.legend(handles=legend, loc='upper right', fontsize=8)

    img_display = ax.imshow(
        build_image(grid_map),
        interpolation='nearest'
    )

    # How many visited cells to show per frame (controls animation speed)
    step_size = max(1, len(visited_order) // 60)

    def update(frame):
        if frame < len(visited_order) // step_size:
            # Show exploration phase
            shown = visited_order[:frame * step_size]
            img_display.set_data(build_image(grid_map, visited=shown))
        else:
            # Show final path
            img_display.set_data(build_image(grid_map, visited=visited_order, path=path))
        return [img_display]

    total_frames = (len(visited_order) // step_size) + 20
    anim = animation.FuncAnimation(
        fig, update,
        frames=total_frames,
        interval=50,
        blit=True
    )

    if save_path:
        anim.save(save_path, writer='pillow', fps=20)
        print(f"  Saved animation → {save_path}")

    plt.tight_layout()
    plt.show()
    plt.close()

def plot_comparison(grid_map, results):
    """
    Show Dijkstra vs A* side by side with stats.
    results = [{'name':..., 'path':..., 'visited':..., 'time':...}, ...]
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle('Path Planning Algorithm Comparison', fontsize=16, fontweight='bold')

    for ax, result in zip(axes, results):
        img = build_image(grid_map, visited=result['visited'], path=result['path'])
        ax.imshow(img, interpolation='nearest')
        ax.axis('off')

        stats = (f"{result['name']}\n"
                 f"Path length:    {len(result['path'])} steps\n"
                 f"Cells explored: {len(result['visited'])}\n"
                 f"Time:           {result['time']*1000:.2f} ms")
        ax.set_title(stats, fontsize=11, family='monospace', loc='left')

    plt.tight_layout()
    plt.savefig('results/comparison.png', dpi=150, bbox_inches='tight')
    print("  Saved comparison → results/comparison.png")
    plt.show()
    plt.close()