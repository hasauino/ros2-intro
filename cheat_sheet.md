
# ROS 2 Cheat Sheet



## 🔧 Workspace Setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```



## 🛠️ Build & Source

```bash
colcon build
source install/setup.bash  # or .zsh, depending on your shell
```



## 📦 Package Management

```bash
ros2 pkg list
ros2 pkg prefix <package_name>
ros2 pkg create --build-type ament_python <package_name>
ros2 pkg create --build-type ament_cmake <package_name>
```



## 🚀 Running Nodes

```bash
ros2 run <package_name> <executable_name>
```



## 📚 Launch Files

```bash
ros2 launch <package_name> <file.launch.py>
```



## 📡 Topics

```bash
ros2 topic list
ros2 topic echo <topic_name>
ros2 topic info <topic_name>
ros2 topic pub <topic_name> <msg_type> '{data: value}'
```



## 🧪 Services

```bash
ros2 service list
ros2 service call <service_name> <srv_type> '{args}'
ros2 service type <service_name>
```



## ⚙️ Parameters

```bash
ros2 param list
ros2 param get <node_name> <param_name>
ros2 param set <node_name> <param_name> <value>
```



## 👀 Nodes

```bash
ros2 node list
ros2 node info <node_name>
```



## 🧩 Interface Info

```bash
ros2 interface list
ros2 interface show <type>
```



## 🔁 Lifecycle

```bash
ros2 lifecycle list
ros2 lifecycle get <node_name>
ros2 lifecycle set <node_name> configure
ros2 lifecycle set <node_name> activate
```



## 📦 Build System Tools

```bash
rosdep install --from-paths src -y --ignore-src
rosdep update
```

---

💡 **Tip**: Always remember to source your workspace:
```bash
source install/setup.bash
```
