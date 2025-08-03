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




## :link: Links

- ROS Jazzy [docs](https://docs.ros.org/en/jazzy) 

- Foxglove [app](https://app.foxglove.dev)