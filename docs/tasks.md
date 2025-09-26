# :dart: Tasks



### Prepare

- start the docker container. We will keep this container running.

- In another terminal, enter the container

  ```bash
  docker exec -it rosws /bin/bash       # or use dox alias
  ```

- Build the workspace

  from inside the container:

  ```bash
  cd ~/ros2_ws/
  colcon build --symlink-install
  ```

- Source setup.sh

  ```bash
  source ~/ros2_ws/install/setup.sh
  ```

- Launch tabit turtlebot file

  ```bash
  ros2 launch tabit turtlebot.launch.py nav:=true
  ```

- Open Webots from host. Open the file generated inside `ws/webots`

- Open rviz in another terminal.



### Task 1: Create a subscriber

- Create a node that subscribes to `/clicked_point` topic of type: [geometry_msgs/PointStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html). And log the coordinates on the terminal. 
- For thar use the  `ws/src/tabit/tabit/exercise.py` file, and complete:
  - **TODO [1]**
  - **TODO [2]**

- Use rviz to publish the point.
- Run it using `ros2 run tabit exercise` command



### Task 2: Read the Map

- In this task, you will read an Occupancy Grid Map. In order to do the task, you need to check it's data format. [This page](https://github.com/a2s-institute/foundations_course/blob/master/content/ros/nodes_topics_messages/home_work/occupancy_grid_maps.md) contains info about that.
- Check [OccupancyGrid Msg](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html). The `info` field contains information about grid width (number of cells), resolution (cell size), etc..

- Extend `exercise.py` to show the occupancy value of a clicked point. For that, use the already created **Map** class inside `ws/src/tabit/tabit/utils/map.py` and complete:
  - **TODO [3]** 



### :fire: Bonus Task: Path Planning

Extend `excersie` further to plan a path from a fixed point (`[0.0, 0.0]`) in the map frame, to a given point on the `/clicked_point` topic. Use BFS search algorithm. The node should publish the path found to `/our_plan` topic. To do that, follow complete:

- **TODO [4]**

For implementing BFS, a useful data structure you can use is Python queue:

```python
from collections import deque

# Create a queue
q = deque()

# Enqueue (append to the right)
q.append(10)
q.append(20)

# Dequeue (pop from the left)
item = q.popleft()   # 10

# Peek at the front (without removing)
front = q[0]         # 20

# Check if empty
is_empty = len(q) == 0

# Use as a stack (LIFO)
q.append(30)
last = q.pop()       # 30

# Bounded queue (max size)
q = deque(maxlen=3)
q.append(1); q.append(2); q.append(3)
q.append(4)          # 1 gets discarded
```

