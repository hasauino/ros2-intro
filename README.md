# ROS2 Intro



## 🛠️ Prep 

- Clone the repo

- Build docker image

  ```bash
  docker-compose build
  ```

- Add `dox` bash alias so we can quickly enter ros2 container

  ```bash
  echo 'alias dox="docker exec -it rosws /bin/bash"' >> ~/.bashrc
  ```

- Install [Webots](https://cyberbotics.com/#download) simulator on the host (not inside docker). 

  

## :rocket: Getting Started

- Start docker container

  ```bash
  docker-compose up --detach
  ```

- Use ROS commands within the container

  ```bash
  hassan@machine:~$ dox
  ros@xxxxx:~$ ros2 --help
  ```

  

## 🤖 Simulation

- Turtlebot

  ```bash
  ros2 launch tabit turtlebot.py
  ```

- Tesla

  ```bash
  ros2 launch tabit tesla.py
  ```

  

## 📌 Examples

- Publish a stamped twist msg

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'base_link'
  },
  twist: {
    linear: {x: 0.1, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.3}
  }
}"
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
    
      

