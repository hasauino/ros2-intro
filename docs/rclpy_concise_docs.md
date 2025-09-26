# rclpy (ROS 2 Python)

This is a quick, working reference for the core pieces you use all the time: initializing and shutting down, node, publishers, subscribers, timers, and spinning.

---

## Initialize & shutdown

```python
import rclpy

rclpy.init(args=None)      # Call once before creating nodes
# ...
rclpy.shutdown()           # Call once when you’re done
```

- `args`: pass `sys.argv`-like list to let ROS parse CLI remappings. Most apps just use the default `None`.

---

## Node: create, spin, destroy

```python
from rclpy.node import Node
import rclpy

def main():
    rclpy.init()
    node = Node('minimal_node')     # name must be unique in the graph

    rclpy.spin(node)                 # process callbacks until Ctrl+C / shutdown
    node.destroy_node()              # free resources tied to the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Other spins:
- `rclpy.spin_once(node, timeout_sec=None)`: single iteration (useful in custom loops).
- `rclpy.spin_until_future_complete(node, future, timeout_sec=None)`: block until a future completes (e.g., service/client).

Logging:
```python
node.get_logger().info("hello")
```

---

## Publishers

```python
from std_msgs.msg import String

# last arg is the queue size
pub = node.create_publisher(String, 'chatter', 10)

msg = String()
msg.data = "hi"
pub.publish(msg)
```


---

## Subscriptions

```python
from std_msgs.msg import String

def cb(msg: String):
    node.get_logger().info(f"heard: {msg.data}")

sub = node.create_subscription(String, 'chatter', cb, 10)
```

Notes:
- Callback signature is `callback(message)`.
- Keep a reference to the subscription (`sub`) so it isn’t garbage-collected.

---

## Timers

```python
# Call a function every 0.5 seconds
def on_timer():
    node.get_logger().info("tick")

timer = node.create_timer(0.5, on_timer)
```

- Period is in **seconds** (float).
- Timer returns a `Timer` object; keep a reference if you may cancel it later:
  ```python
  timer.cancel()   # stop
  timer.reset()    # reset next call time
  ```

---

## Putting it together — minimal talker & listener

**talker.py**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.i = 0
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        msg = String()
        msg.data = f"hello {self.i}"
        self.pub.publish(msg)
        self.get_logger().info(f"published: {msg.data}")
        self.i += 1

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**listener.py**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'chatter', self.cb, 10)

    def cb(self, msg: String):
        self.get_logger().info(f"heard: {msg.data}")

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run in separate terminals:
```
ros2 run <your_pkg> talker
ros2 run <your_pkg> listener
```

---



---

## Graceful shutdown & Ctrl+C

If you need custom cleanup:
```python
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    node.get_logger().info('Interrupted')
finally:
    node.destroy_node()
    rclpy.shutdown()
```

---

## TL;DR cheat-sheet

- `rclpy.init() / rclpy.shutdown()`
- `node = Node('name')`
- `pub = node.create_publisher(MsgType, 'topic', qeueu_size)`
- `sub = node.create_subscription(MsgType, 'topic', callback, qeueu_size)`
- `timer = node.create_timer(period_sec, callback)`
- `rclpy.spin(node)` → process callbacks
- `node.destroy_node()` when done
