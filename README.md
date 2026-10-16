# Reactive Laser-Based Roaming for Mapping with SLAM Toolbox

A 4-wheel differential drive robot roams an unknown environment using only a 2D laser scan, while SLAM Toolbox builds the map in the background. Implemented in Python (`rclpy`) on ROS 2 Jazzy, Gazebo Harmonic and Ubuntu 24.04.

> **Note:** This is a reactive baseline, not frontier-based exploration. The robot does not use the map to decide where to go. See [Limitations](#limitations).



## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-xacro \
  ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server python3-numpy \
  python3-colcon-common-extensions
```

## Build

```bash
git clone https://github.com/shanshad/Autonomous_slam_using_nav2_slam_toolbox_and_explore_lite_with_4_wheel_diff_drive_robot.git
cd Autonomous_slam_using_nav2_slam_toolbox_and_explore_lite_with_4_wheel_diff_drive_robot/ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Source `/opt/ros/jazzy/setup.bash` and `ws/install/setup.bash` in every terminal.

```bash
# 1. Gazebo, robot, RViz
ros2 launch my_robot_bringup robot.launch.xml

# 2. SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# 3. Roaming controller
python3 ws/src/my_robot_bringup/scripts/controller.py --ros-args -p use_sim_time:=true
```

Save the map when done:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

## Method

The controller subscribes to `/scan` and publishes `/cmd_vel`. The control loop runs at 10 Hz.

### 1. Scan model

A scan contains $N$ range readings $r_i$ taken at angles

$$\theta_i = \theta_{min} + i\,\Delta\theta, \quad i = 0,\dots,N-1$$

where $\theta_{min}$ is the first beam angle and $\Delta\theta$ is the angular increment. For the lidar used here, $N = 1800$ and $\theta \in [-\pi, \pi]$, so $\theta = 0$ is straight ahead and positive angles are to the left (counter-clockwise).

Angle to index conversion:

$$i(\theta) = \left\lfloor \frac{\theta - \theta_{min}}{\Delta\theta} \right\rfloor$$

### 2. Range cleaning

Invalid readings (`NaN`, `inf`) mean nothing was detected, so they are replaced by the maximum range:

$$\tilde r_i = \begin{cases} r_{max} & \text{if } r_i \text{ is NaN or } \infty \\ r_i & \text{otherwise} \end{cases}$$

### 3. Front clearance

The distance to the nearest obstacle in a $\pm 30^\circ$ cone ahead:

$$d_{front} = \min_{i \in \mathcal{I}_{front}} \tilde r_i, \qquad \mathcal{I}_{front} = \{\, i(-30^\circ), \dots, i(+30^\circ) \,\}$$

### 4. Emptiest direction

Over the front window $\mathcal{W} = \{ i(-120^\circ), \dots, i(+120^\circ) \}$, the ranges are smoothed with a moving average of $K = 80$ samples (about $16^\circ$), so that a single long reading through a narrow crack cannot win:

$$s_i = \frac{1}{K} \sum_{j=-K/2}^{K/2-1} \tilde r_{i+j}, \quad i \in \mathcal{W}$$

The emptiest direction is where the smoothed range is largest:

$$i^* = \arg\max_{i \in \mathcal{W}} s_i, \qquad \theta^* = \theta_{min} + i^*\,\Delta\theta$$

### 5. Turn direction

$$\sigma = \begin{cases} +1 \text{ (turn left)} & \theta^* \ge 0 \\ -1 \text{ (turn right)} & \theta^* < 0 \end{cases}$$

### 6. Control law (two-state machine with hysteresis)

| State | Command | Transition |
|---|---|---|
| `DRIVE` | $v = 0.15$ m/s, $\omega = 0$ | to `TURN` when $d_{front} < d_{stop}$; $\sigma$ is chosen once at this moment |
| `TURN` | $v = 0$, $\omega = 0.4\,\sigma$ rad/s | to `DRIVE` when $d_{front} > d_{clear}$ |

with $d_{stop} = 0.6$ m and $d_{clear} = 1.0$ m. Because $d_{clear} > d_{stop}$, the robot cannot flicker between states near the threshold, and holding $\sigma$ fixed during the turn prevents the direction from flipping mid-rotation.

## Parameters

| Parameter | Value | Meaning |
|---|---|---|
| Control rate | 10 Hz | Timer period 0.1 s |
| Forward speed $v$ | 0.15 m/s | Speed while driving |
| Turn rate $\omega$ | 0.4 rad/s | Speed while turning |
| Front cone | $\pm 30^\circ$ | Used for $d_{front}$ |
| Search window | $\pm 120^\circ$ | Used for $\theta^*$ |
| Smoothing window $K$ | 80 samples | About $16^\circ$ |
| $d_{stop}$ | 0.6 m | Start turning |
| $d_{clear}$ | 1.0 m | Stop turning |

## Limitations

- **No coverage guarantee.** The robot ignores the map, so it can loop inside a room and may miss exits.
- **Pose-free.** It uses no odometry or localization, so it cannot return to its start or know what is already mapped.
- **Only the front cone is checked** for safety, so it can clip obstacles on its sides.
- **Argmax of smoothed range** favors far corners over doorways, so it is used only to choose the turn direction, not to steer.

## Future Work

- Stop when the mapped area reaches a set threshold: $A = n_{known}\,\rho^2$, where $n_{known}$ is the number of non-unknown cells in `/map` and $\rho$ is the map resolution.
- Return to the start pose using A* on the occupancy grid.
- Randomized turn angles and stuck detection to reduce looping.
- Quantitative comparison against Nav2 + explore_lite (mapped area vs time, distance travelled).

## Repository Structure

```
ws/src/
├── my_robot_description/   # URDF/Xacro robot model
└── my_robot_bringup/       # Launch files and controller script
```
