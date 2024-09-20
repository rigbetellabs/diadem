## WaypointNavigator Node

### Overview

This Python script is a ROS2 node for controlling a vehicle using MAVROS and waypoints. The `WaypointNavigator` node interacts with a Pixhawk flight controller, enabling functionalities such as setting waypoints, taking off, and returning to launch (RTL) mode.

### Functions

#### `__init__(self)`
- Initializes the ROS2 node and various service clients.
- Sets the stream rate for MAVROS topics.
- Subscribes to relevant topics to monitor the vehicle's position and waypoint status.
- Logs the initialization status and sets the stream rate for MAVROS topics.

#### `waypoint_reached_callback(self, msg)`
- Callback function for the `/mavros/mission/reached` topic.
- Logs when the final waypoint in the mission is reached.
- Sets `self.mission_complete` to `True` when the mission is completed.

#### `global_position_callback(self, data)`
- Callback function for the `/mavros/global_position/global` topic.
- Logs the vehicle's global position.
- Checks for a valid GPS fix and updates `self.global_position`.

#### `takeoff(self)`
- Sends a takeoff command to the vehicle.
- Logs the takeoff status.

#### `setmode(self, mode)`
- Sets the vehicle's mode (e.g., `GUIDED`, `AUTO`, `RTL`).
- Logs the mode change status.

#### `arm_vehicle(self, arm)`
- Arms or disarms the vehicle.
- Logs the arming/disarming status.

#### `goToLocation(self, lat, long)`
- Plans a mission with waypoints and sends it to the vehicle.
- Clears existing waypoints.
- Creates a list of waypoints including a takeoff waypoint and a target location waypoint.
- Pushes the waypoint list to the vehicle.
- Arms the vehicle and sets it to `AUTO` mode.
- Initiates takeoff and monitors the mission progress.
- Logs the mission progress and completion status.

#### `run_test(self)`
- Test function that sends a waypoint plan to the vehicle.
- Sets the vehicle to `RTL` mode after the mission is completed.


### Functions Description

- **Initialization (`__init__`):**
  - Creates service clients for setting stream rate, arming, setting mode, taking off, pushing waypoints, and clearing waypoints.
  - Subscribes to topics for global position and waypoint reached events.
  - Sets the MAVROS stream rate to 10 Hz.

- **Waypoint Reached Callback (`waypoint_reached_callback`):**
  - Checks if the current waypoint is the last one in the mission.
  - Logs the mission completion status.

- **Global Position Callback (`global_position_callback`):**
  - Logs the vehicle's latitude, longitude, and altitude.
  - Checks if the GPS fix is valid and updates the global position.

- **Takeoff (`takeoff`):**
  - Sends a takeoff request to the vehicle.
  - Logs the takeoff status.

- **Set Mode (`setmode`):**
  - Sends a request to set the vehicle's mode.
  - Logs the mode change status.

- **Arm Vehicle (`arm_vehicle`):**
  - Sends a request to arm or disarm the vehicle.
  - Logs the arming or disarming status.

- **Go To Location (`goToLocation`):**
  - Disarms the vehicle and sets it to `GUIDED` mode.
  - Clears any existing waypoints.
  - Creates and pushes a new waypoint list to the vehicle.
  - Arms the vehicle and sets it to `AUTO` mode.
  - Initiates takeoff and monitors mission progress.
  - Logs the mission status and completion.

- **Run Test (`run_test`):**
  - Sends a waypoint plan to a predefined location.
  - Sets the vehicle to `RTL` mode after the mission is completed.

