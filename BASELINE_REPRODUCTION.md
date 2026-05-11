# Baseline Reproduction Notes

## Environment

- Repository: `jongco22/LLM_DWA`
- ROS: Humble
- Simulator: Gazebo
- Navigation stack: Nav2 / DWA local planner
- Robot: TurtleBot3 `waffle`
- Mode: text-based LLM waypoint generation

## Baseline Scenario 1

- World: `jh_world`
- Start: `(-7, -6)`
- Goal: `(-7, 5)`
- Trials: 3
- Result: navigation succeeded in all 3 trials
- Waypoint consistency: same waypoint sequence generated in all 3 trials

Generated waypoints:

```json
{
  "waypoints": [
    [-7, -6],
    [-5, 0.5],
    [-2, -4],
    [2, -0.5],
    [6.0, 1.0],
    [-7, 5]
  ]
}
```

## Reproduction Steps

1. Start the Docker container.

   ```bash
   docker compose up -d
   docker exec -it llm_dwa bash
   ```

2. Build the ROS workspace.

   ```bash
   cd /workspace/ros2_ws
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=waffle
   colcon build
   source install/setup.bash
   ```

3. Launch Gazebo.

   ```bash
   ros2 launch turtlebot3_gazebo jh_world.launch.py
   ```

4. Launch Nav2.

   ```bash
   ros2 launch nav2_bringup bringup_launch.py map:=/workspace/ros2_ws/jh_world/my_map.yaml use_sim_time:=True
   ```

5. Launch RViz and set the initial pose.

   ```bash
   ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
   ```

6. Parse obstacle information.

   ```bash
   python3 /workspace/ros2_ws/src/turtlebot3_llm_nav2/scripts/parse_world.py /workspace/ros2_ws/src/turtlebot3_llm_nav2/data/jh_world.world /workspace/ros2_ws/src/turtlebot3_llm_nav2/data/obstacle.json
   ```

7. Generate LLM waypoints.

   ```bash
   python3 /workspace/ros2_ws/src/turtlebot3_llm_nav2/scripts/run_llm_pipeline.py
   ```

   Inputs:

   ```text
   Start (x y): -7 -6
   Goal (x y): -7 5
   ```

8. Execute navigation through generated waypoints.

   ```bash
   ros2 run turtlebot3_llm_nav2 waypoint_navigator.py
   ```

## Reproduction Notes

- Installed missing ROS dependencies inside the Docker container:

  ```bash
  apt install -y ros-humble-turtlebot3-msgs ros-humble-turtlebot3
  ```

- Created missing parameter directory for `turtlebot3_fake_node`:

  ```bash
  mkdir -p /workspace/ros2_ws/src/turtlebot3_simulations/turtlebot3_fake_node/param
  ```

- Installed Python OpenAI client compatible with the repository's legacy API usage:

  ```bash
  pip3 install openai==0.28.1
  ```

- Created `/workspace/ros2_ws/.env` with `OPENAI_API_KEY`.
- Converted `waypoint_navigator.py` line endings from CRLF to LF after `/usr/bin/env: 'python3\r'` execution failure.

## Next Research Direction

The next extension is failure-aware LLM-DWA replanning for dynamically blocked environments. The main idea is to invoke the LLM as a high-level waypoint replanner only when persistent dynamic blockage causes Nav2/DWA local planner failure.
