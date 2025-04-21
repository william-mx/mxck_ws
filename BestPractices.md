# Best Practices for ROS 2 Development

## Preventing Duplicate Nodes and Handling Crashes

To prevent running nodes twice, ensure each `Node` action in your launch file is intentional and unique.

For automatic respawning on crashes:

```python
vesc = Node(
    package = 'vesc_driver',
    name = 'vesc_driver_node',
    executable = 'vesc_driver_node',
    parameters = [vesc_config],
    respawn=True,
    respawn_delay=20.0
)
```

Checking if a node is running from within a launch file using `ros2 node list`:

```python
import subprocess

# Get list of currently running nodes
result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True)

# Get node names and remove the leading slash
running_nodes = [node.lstrip('/') for node in result.stdout.strip().split('\n')] if result.stdout else []

node_names = {
    'rc_to_joy': rc2joy,
    'joy_to_ackermann': joy2ackermann,
    'ackermann_to_vesc': ackermann2vesc,
    'vesc_driver_node': vesc
}
for name, node in node_names.items():
    if name not in running_nodes:
        ld.add_action(node)
```

Checking from within a node:

```python
# Filter nodes with the same name
names = [name for name in node.get_node_names() if name == 'my_node_name']
# Count nodes with this name
if len(names) > 0:
    node.get_logger().error(f'Another {node_name_to_check} node is already running! Exiting...')
    sys.exit(1)
```

**I recommend using the launch file method (checking with `ros2 node list` within the launch file) to prevent running duplicate nodes.**

**Reasoning:** The launch file provides a central point of control for managing all nodes in your system. This is crucial because:

* You can **overwrite the node name** specified in the node's code directly within the launch file. This means an in-node check based on a hardcoded name might not detect duplicates launched with different names via launch files.
* More importantly, when dealing with **installed nodes (e.g., drivers for Realsense cameras or other hardware)**, you often cannot modify their internal code to implement duplicate detection. The launch file becomes the only place where you can attempt to check for existing instances of these nodes and conditionally launch them or prevent duplicate launches.

While this launch file method has limitations like potential race conditions, it offers a necessary level of control, especially for managing external, unmodifiable nodes in your ROS 2 system.
