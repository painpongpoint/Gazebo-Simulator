# Gazebo Car Control Simulator

A Gazebo Harmonic driving simulation with a hatchback, physical wheel and body
collisions, and a small desktop control panel for driving and steering.

## Requirements and startup

Requires Ubuntu 24.04, Gazebo Harmonic (Gazebo Sim 8) with its Python transport
and message bindings, and `python3-tk`. Windows launchers use a WSL distribution
named `Ubuntu-24.04`. The model assets are included in this repository.

```sh
git clone https://github.com/painpongpoint/Gazebo-Simulator.git
cd Gazebo-Simulator
python3 launch_world.py
```

In a second Ubuntu terminal in the same directory:

```sh
python3 car_controls.py
```

On Windows, double-click **Open Saved Driving World.cmd**, then **Start Car
Controls.cmd**. The launchers use their own folder, so the checkout can be moved.
The world launcher sets Gazebo's resource search paths for the bundled meshes.

## Controls

- Hold **Forward** or **Backward** to drive. Release to brake.
- Hold **Left** or **Right** to drive forward in a turning arc.
- Arrow keys work while the control window has focus. Combine Up/Down with
  Left/Right to steer while driving forward or reversing.
- Click **Stop**, press Space or Escape, switch away from the window, or close it
  to brake. The speed slider sets the requested driving speed.

The car now has a dynamic rigid body, four rolling wheels, front steering joints,
and Gazebo's Ackermann steering controller. The controls send velocity commands;
normal driving never teleports the car. Body and tire collision shapes interact
with other collidable entities. Movable shapes can be pushed; fixed obstacles
block the car. Body collisions use two approximate boxes, not the detailed visual
mesh. Turning requires movement, as on a car. Braking takes a short distance.

The upgraded world is saved in `world_drivable.sdf`. After closing the current
Gazebo session, double-click **Open Saved Driving World.cmd**, then
**Start Car Controls.cmd**. Do not run two copies of the `shapes` world at once.
The control launcher alone does not start Gazebo.

`build_car.py` creates the local model from the cached Fuel hatchback mesh;
`upgrade_car.py` replaces the car in the running world and keeps a world backup
under `backups/`. `test_car_physics.py` creates temporary models away from the
main scene, checks forward/reverse motion, left/right steering, fixed-wall blocking,
and dynamic-box pushing, then removes the temporary models.

Controller reference: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html

See [THIRD_PARTY.md](THIRD_PARTY.md) for model attribution. The live integration
tests have verified forward/reverse motion, steering in both directions,
wall blocking, and pushing a movable cube on Gazebo Sim 8.15.0.
