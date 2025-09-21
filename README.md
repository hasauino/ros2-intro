# ROS2 Intro



## 🛠️ Prep 

- Clone the repo

- Add `dox` bash alias so we can quickly enter ros2 container

  ```bash
  echo 'alias dox="docker exec -it rosws /bin/bash"' >> ~/.bashrc
  ```

- Install [Webots](https://cyberbotics.com/#download) (latest version - R2025a) simulator on the host (not inside docker). 

  

## :rocket: Getting Started

- Start docker container

  ```bash
  docker-compose up
  ```

- Use ROS commands within the container

  ```bash
  hassan@machine:~$ dox
  ros@xxxxx:~$ ros2 --help
  ```

  

## 🤖 Simulation

- Launch a ROS simulation

    - Turtlebot
      ```bash
      ros2 launch tabit turtlebot.py
      ```

    - Tesla
      ```bash
      ros2 launch tabit tesla.py
      ```


- Open Simulation world in Webots (from host, :file_folder: `ws/webots/<sim file>`)



## 📌 Examples

### :snake: Python

- Talker node

```bash
ros2 run tabit talker
```

- Listener node

```bash
ro2 run tabit listener
```

- Robot move in circle

```bash
ros2 run tabit robot_commander
```



### NodeJs

Got to  :file_folder: [demo_js](ws/src/demo_js)



## CLI

- echo a topic

```bash
ros2 topic echo /my_topic
```

- Publish a stamped twist msg

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" --rate 10
```

- Keyboard teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```




## :link: Links

- ROS Jazzy [docs](https://docs.ros.org/en/jazzy) 

- Cheat [sheet](cheat_sheet.md)

- Foxglove [app](https://app.foxglove.dev)

- ROS client libraries 

    - [Python](https://docs.ros.org/en/jazzy/p/rclpy/) :star:

    - [C++](https://docs.ros.org/en/jazzy/p/rclcpp/) :star:
    
    - [Rust](https://github.com/adnanademovic/rosrust)
    
    - [NodeJs](https://github.com/RobotWebTools/rclnodejs)
    
- ROS2 on GitHub: https://github.com/ros2/

