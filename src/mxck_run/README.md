# MXcarkit - Robot Description Launch

## Overview
This repository provides the **MXcarkit** URDF robot description and a ROS 2 launch file to publish its transforms using `robot_state_publisher`.

## What Does This Launch File Do?
The launch file:
- **Loads the URDF** from the package source directory (`/humble_ws/src/mxck_run/urdf/mxcarkit.urdf`).
- **Publishes the robot description** to `/robot_description`.
- **Publishes the static TF tree** of MXcarkit using `robot_state_publisher`.

## Topics Published
After launching, three key topics are available:
| Topic               | Description |
|---------------------|-------------|
| `/robot_description` | The URDF model of the robot as a string. Used by `robot_state_publisher`. |
| `/tf_static`        | Contains all **static** transforms (links connected by fixed joints). This is important for visualization. |
| `/tf`              | Normally contains **dynamic** transforms for moving joints, but **MXcarkit only has static joints**, so this topic is **empty** in our case. |

## Why `/tf` Is Empty
Since **all joints in MXcarkit are `fixed`**, there are no moving parts, meaning:
- The transform tree is **fully static**.
- `/tf_static` contains all transforms.
- `/tf` remains **empty** because no dynamic transformations exist.

### **How Could We Introduce Movement?**
If we wanted to make the vehicle move in 3D, we could:
1. **Introduce a `world` link** as a parent frame.
2. **Use a moving joint** (e.g., `floating` or `continuous`).
3. **Publish real-time transforms** via a localization system or `tf2_ros`.

## Visualizing in Foxglove
By default, **Foxglove Studio does not display `/tf_static`**. To see the robot, you **must whitelist `/tf_static`**:
