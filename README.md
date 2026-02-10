## 🐳Docker
### Make docker container
```
git clone https://github.com/jongco22/LLM_DWA.git
cd LLM_DWA
docker compose up -d
```

### Excute docker container
```
docker exec -it llm_dwa bash
```

## Excution(Text)
### Start Gazebo
```
ros2 launch turtlebot3_gazebo <launch_file_name>.launch.py
```

### nav2 bringup
```
ros2 launch nav2_bringup bringup_launch.py map:=<map.yaml path> use_sim_time:=True
```

### Rviz display
```
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### parsing obstacles
```
python3 /workspace/ros2_ws/src/turtlebot3_llm_nav2/scripts/parse_world.py /workspace/ros2_ws/src/turtlebot3_llm_nav2/data/jh_world.world /workspace/ros2_ws/src/turtlebot3_llm_nav2/data/obstacle.json
```

### input start, goal
```
python3 /workspace/ros2_ws/src/turtlebot3_llm_nav2/scripts/run_llm_pipeline.py
```

### Nav2 (Navigation)
```
ros2 run turtlebot3_llm_nav2 waypoint_navigator.py
```

### Localization
2d pose estimation in Rviz2

## Excution(Image)
### Start Gazebo
```
ros2 launch turtlebot3_gazebo <launch_file_name>.launch.py
```

### nav2 bringup
```
ros2 launch nav2_bringup bringup_launch.py map:=<map.yaml path> use_sim_time:=True
```

### Rviz display

```
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### input start, goal
```
python3 /workspace/ros2_ws/src/turtlebot3_llm_nav2/scripts/run_image_pipeline.py
```

### Nav2 (Navigation)
```
ros2 run turtlebot3_llm_nav2 waypoint_navigator.py
```

### Localization
2d pose estimation in Rviz2

## VNC(gui)
[http://localhost:6080/vnc.html](http://localhost:6080/vnc.html)
