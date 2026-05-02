import mujoco
import numpy as np
import matplotlib.pyplot as plt

# MuJoCo XML: a differential drive robot (two wheels + a body)
ROBOT_XML = """
<mujoco model="mobile_robot">
  <option timestep="0.01" gravity="0 0 -9.81"/>

  <worldbody>
    <geom name="floor" type="plane" size="15 15 0.1"
          rgba="0.9 0.9 0.9 1" friction="1 0.5 0.5"/>

    <body name="robot" pos="0 0 0.05">
      <freejoint/>
      <geom type="box" size="0.15 0.1 0.05" rgba="0.2 0.5 0.9 1" mass="1"/>

      <!-- Left wheel -->
      <body name="left_wheel" pos="0 0.12 0">
        <joint name="left_wheel_joint" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" size="0.05 0.02" rgba="0.2 0.2 0.2 1"
              euler="90 0 0" mass="0.1"/>
      </body>

      <!-- Right wheel -->
      <body name="right_wheel" pos="0 -0.12 0">
        <joint name="right_wheel_joint" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" size="0.05 0.02" rgba="0.2 0.2 0.2 1"
              euler="90 0 0" mass="0.1"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <velocity joint="left_wheel_joint"  gear="1" kv="10"/>
    <velocity joint="right_wheel_joint" gear="1" kv="10"/>
  </actuator>
</mujoco>
"""

def path_to_world(path, grid_map, scale=0.5):
    """
    Convert grid (row, col) coordinates to MuJoCo world (x, y) coordinates.
    Grid origin is top-left; world origin is center.
    """
    cx = grid_map.cols / 2
    cy = grid_map.rows / 2
    world_path = []
    for (r, c) in path:
        x =  (c - cx) * scale
        y = -(r - cy) * scale   # flip y axis
        world_path.append((x, y))
    return world_path

def run_simulation(path, grid_map, title="Robot Simulation"):
    """
    Drive the robot along the path using a P controller.
    Records and plots the actual trajectory taken.
    """
    if not path:
        print("No path to simulate!")
        return

    model = mujoco.MjModel.from_xml_string(ROBOT_XML)
    data  = mujoco.MjData(model)

    waypoints   = path_to_world(path, grid_map)
    wp_index    = 0
    actual_traj = []

    # Place robot at the first waypoint
    start_x, start_y = waypoints[0]
    data.qpos[0] = start_x
    data.qpos[1] = start_y
    data.qpos[2] = 0.05

    print(f"\nSimulating robot following {len(waypoints)} waypoints...")

    for step in range(5000):
        if wp_index >= len(waypoints):
            break

        # Current robot position
        rx, ry = data.qpos[0], data.qpos[1]
        actual_traj.append((rx, ry))

        # Target waypoint
        tx, ty = waypoints[wp_index]
        dx, dy = tx - rx, ty - ry
        dist   = np.sqrt(dx**2 + dy**2)

        # Move to next waypoint if close enough
        if dist < 0.1:
            wp_index += 1
            continue

        # P controller — steer toward waypoint
        target_angle = np.arctan2(dy, dx)
        robot_angle  = data.qpos[3] if len(data.qpos) > 3 else 0.0
        angle_error  = target_angle - robot_angle

        speed       = min(2.0, dist * 3.0)      # slow down near waypoint
        turn        = angle_error * 2.0

        data.ctrl[0] = speed - turn   # left wheel
        data.ctrl[1] = speed + turn   # right wheel

        mujoco.mj_step(model, data)

    print(f"  Reached {wp_index}/{len(waypoints)} waypoints")
    _plot_trajectory(waypoints, actual_traj, grid_map, title)

def _plot_trajectory(waypoints, actual_traj, grid_map, title):
    """Plot the planned path vs actual trajectory taken"""
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Planned path
    wx = [w[0] for w in waypoints]
    wy = [w[1] for w in waypoints]
    ax.plot(wx, wy, 'o--', color='orange', linewidth=2,
            markersize=4, label='Planned path', zorder=3)

    # Actual trajectory
    if actual_traj:
        ax.plot([t[0] for t in actual_traj],
                [t[1] for t in actual_traj],
                '-', color='steelblue', linewidth=1.5,
                alpha=0.7, label='Actual trajectory')

    # Start and goal markers
    ax.plot(wx[0],  wy[0],  's', color='green', markersize=12,
            label='Start', zorder=5)
    ax.plot(wx[-1], wy[-1], '*', color='red',   markersize=14,
            label='Goal',  zorder=5)

    ax.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig('results/trajectory.png', dpi=150, bbox_inches='tight')
    print("  Saved trajectory → results/trajectory.png")
    plt.show()
    plt.close()