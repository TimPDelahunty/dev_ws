# ROS2 Pan-Tilt Camera Control System Documentation

## 🎉 SUCCESS! Complete Pan-Tilt Camera System Working!

✅ **Robot Movement** - Pan/tilt joints working in Gazebo  
✅ **Keyboard Control** - Real-time W/A/S/D control implemented  
✅ **Camera Integration** - Live camera feed from Gazebo  
✅ **Object Tracking** - Automatic object following capability  

This document explains the complete system setup, usage, and troubleshooting.

---

## � Quick Start Guide

### 1. Launch the Complete System
```bash
# Navigate to workspace and source environment
cd /home/tim/dev_ws
source install/setup.bash

# Launch Gazebo simulation with robot, camera, and all bridges
ros2 launch my_bot launch_sim_arti.launch.py
```

### 2. Control the Robot (Choose one option below)

#### Option A: Keyboard Control (Manual Control)
```bash
# In a NEW terminal:
cd /home/tim/dev_ws
source install/setup.bash
python3 src/my_bot/keyboard_control_fixed.py

# Controls:
# W/S = Tilt Up/Down
# A/D = Pan Left/Right  
# R = Reset to center
# Q = Quit
```

#### Option B: Object Tracking (Automatic Control)
```bash
# In a NEW terminal:
cd /home/tim/dev_ws
source install/setup.bash
python3 src/my_bot/object_tracker.py

# Automatically tracks orange/red objects in camera view
```

### 3. View Camera Feed
```bash
# In a NEW terminal:
cd /home/tim/dev_ws
source install/setup.bash
ros2 run rqt_image_view rqt_image_view

# Then select '/camera/image_raw' from the dropdown menu
```

### 4. Monitor System Status
```bash
# Check if camera is publishing (should show image data):
ros2 topic echo /camera/image_raw --once

# Check joint positions:
ros2 topic echo /joint_states

# List all available topics:
ros2 topic list

# Check joint commands being sent:
ros2 topic echo /pan_platform_joint/command
ros2 topic echo /tilt_platform_joint/command
```

---

## �📊 Data Flow: ROS2 → Gazebo

### Overview
```
[Command Source] → [ROS2 Bridge] → [Gazebo Physics] → [Visual Update]
```

### Detailed Flow

#### 1. **Command Generation**
- **Source**: Python scripts (`gentle_joint_commander.py`, `direct_joint_commander.py`)
- **Message Type**: `std_msgs/msg/Float64`
- **Topics**: 
  - `/pan_platform_joint/command`
  - `/tilt_platform_joint/command`
- **Command**: Position values in radians

#### 2. **ROS2-Gazebo Bridge**
- **File**: `config/gz_bridge.yaml`
- **Function**: Converts ROS2 topics to Gazebo topics
- **Mapping**:
  ```yaml
  ROS2: /pan_platform_joint/command (std_msgs/msg/Float64)
    ↓
  Gazebo: /model/tracker_robot/joint/pan_platform_joint/cmd_pos (gz.msgs.Double)
  ```

#### 3. **Gazebo Joint Controllers**
- **File**: `description/ros2_control.xacro`
- **Plugin**: `gz-sim-joint-position-controller-system`
- **Function**: Receives position commands and moves joints
- **Control Loop**: PID controller with gains (P=1.0, I=0.01, D=0.05)

#### 4. **Joint State Feedback**
- **Plugin**: `gz-sim-joint-state-publisher-system`
- **Function**: Publishes current joint positions back to ROS2
- **Topic**: `/joint_states`

---

## 🔧 Key Files and Their Roles

### Core Configuration Files
1. **`description/robot.urdf.xacro`**
   - Main robot definition
   - Includes ros2_control.xacro

2. **`description/ros2_control.xacro`** ⭐ CRITICAL
   - gz_ros2_control plugin configuration
   - Joint position controller plugins
   - Joint state publisher plugin

3. **`config/gz_bridge.yaml`** ⭐ CRITICAL
   - Bridges ROS2 ↔ Gazebo topics
   - Must match robot model name (`tracker_robot`)

4. **`launch/launch_sim_arti.launch.py`** ⭐ CRITICAL
   - Spawns robot with correct name
   - Starts bridge with configuration
   - Loads controllers

### Controller Files
5. **`config/controllers.yaml`**
   - ROS2 trajectory controller config (alternative approach)
   
6. **`config/gaz_ros2_ctl_use_sim.yaml`**
   - Simulation time configuration

---

## 🚨 Common Issues and Troubleshooting

### Issue 1: Robot Not Moving
**Symptoms**: Commands sent but no visual movement in Gazebo

**Diagnostic Steps**:
```bash
# 1. Check if commands are being sent
ros2 topic echo /pan_platform_joint/command

# 2. Check if commands reach Gazebo
gz topic --echo --topic /model/tracker_robot/joint/pan_platform_joint/cmd_pos

# 3. Check if Gazebo joint controller topics exist
gz topic -l | grep tracker_robot
```

**Common Causes**:
- ❌ Missing JointPositionController plugin in ros2_control.xacro
- ❌ Wrong model name in bridge configuration
- ❌ Bridge not running or misconfigured

### Issue 2: Robot Goes Crazy/Unstable
**Symptoms**: Robot flies around, falls over, oscillates wildly

**Solutions**:
- ✅ Lower PID gains in JointPositionController
- ✅ Reduce command limits (cmd_max/cmd_min)
- ✅ Send smaller, slower movements
- ✅ Check joint limits in URDF

**Example Good Settings**:
```xml
<p_gain>1.0</p_gain>
<i_gain>0.01</i_gain>
<d_gain>0.05</d_gain>
<cmd_max>10</cmd_max>
<cmd_min>-10</cmd_min>
```

### Issue 3: Name Mismatches
**Symptoms**: Bridge shows no errors but commands don't reach Gazebo

**Check These Names Must Match**:
1. **Robot Model Name**:
   - URDF: `<robot name="tracker_robot">`
   - Launch: `arguments=['-name', 'tracker_robot']`
   - Bridge: `gz_topic_name: "/model/tracker_robot/..."`

2. **Joint Names**:
   - URDF: `<joint name="pan_platform_joint">`
   - Bridge: `/pan_platform_joint/command`
   - Controller: `<joint_name>pan_platform_joint</joint_name>`

### Issue 4: Controllers Not Loading
**Symptoms**: `ros2 control list_controllers` shows no controllers

**Diagnostic Steps**:
```bash
# Check if controller manager is running
ros2 node list | grep controller_manager

# Check controller status
ros2 control list_controllers

# Check spawner logs in launch output
```

**Solutions**:
- ✅ Ensure launch file includes controller spawners
- ✅ Check controller.yaml syntax
- ✅ Verify dependencies in package.xml

---

## ✅ Working Command Examples

### Direct Joint Control (Current Working Method)
```python
# Float64 messages to individual joint command topics
pan_msg = Float64()
pan_msg.data = 3.14  # radians
publisher.publish(pan_msg)
```

### Safe Movement Ranges
- **Pan Joint**: 0 to 6.28 radians (0° to 360°)
- **Tilt Joint**: -0.78 to 1.57 radians (-45° to 90°)
- **Safe increments**: ±0.5 radians max per command

---

## ✅ Working Control Methods

### Method 1: Keyboard Control (✅ **WORKING & COMPLETE**)
- **File**: `keyboard_control_fixed.py`
- **Use**: Manual real-time control with W/A/S/D keys
- **Features**: 
  - Smooth movement with 0.2 radians steps
  - Joint limit enforcement
  - Center reset function
  - Real-time position feedback
- **Pros**: Immediate response, intuitive control
- **Cons**: Requires manual operation

### Method 2: Object Tracking (✅ **WORKING & COMPLETE**)
- **File**: `object_tracker.py`  
- **Use**: Automatic tracking of colored objects
- **Features**:
  - Real-time orange/red object detection
  - Automatic camera positioning
  - Smart deadzone (no jitter in center)
  - Visual feedback with detection boxes
- **Pros**: Autonomous operation, smooth tracking
- **Cons**: Requires specific object colors

### Method 3: Direct Float64 Commands (✅ **WORKING**)
- **Files**: `gentle_joint_commander.py`, `direct_joint_commander.py`
- **Use**: Programmatic control via topics
- **Pros**: Simple, direct control for scripts
- **Cons**: No trajectory planning

### Method 4: Trajectory Controller (⚠️ Configured but not actively used)
- **Topic**: `/pan_tilt_joint_trajectory_controller/joint_trajectory`
- **Message**: `trajectory_msgs/msg/JointTrajectory`
- **Pros**: Smooth trajectories, velocity control
- **Cons**: More complex setup

---

## 📋 Maintenance Checklist

### When Adding New Joints
1. ✅ Add joint to URDF with proper limits
2. ✅ Add joint to ros2_control.xacro (both gz_ros2_control AND JointPositionController)
3. ✅ Add joint bridge mapping to gz_bridge.yaml
4. ✅ Update controller spawners in launch file

### When Robot Behaves Poorly
1. ✅ Check and tune PID gains
2. ✅ Verify joint limits
3. ✅ Check for conflicting controllers
4. ✅ Ensure proper inertia values in URDF

### Regular Debugging Commands
```bash
# Monitor joint states
ros2 topic echo /joint_states

# Check Gazebo topics
gz topic -l

# Check ROS2 topics
ros2 topic list

# Test individual joint
gz topic -t /model/tracker_robot/joint/pan_platform_joint/cmd_pos -m gz.msgs.Double -p 'data: 1.0'
```

---

## 🎯 Success Indicators

### System is Working When:
1. ✅ `ros2 topic echo /pan_platform_joint/command` shows commands
2. ✅ `gz topic --echo --topic /model/tracker_robot/joint/pan_platform_joint/cmd_pos` shows commands
3. ✅ Robot visually moves in Gazebo
4. ✅ `/joint_states` shows changing position values
5. ✅ No error messages in launch output

### Performance Metrics:
- **Response Time**: Commands should affect movement within 0.1-0.5 seconds
- **Stability**: No oscillation or wild movements
- **Accuracy**: Robot reaches commanded positions within ±0.1 radians

---

## 📚 Key Learnings from This Project

1. **gz_ros2_control alone isn't enough** - Need JointPositionController plugins
2. **Name consistency is critical** - Robot name must match everywhere
3. **Bridge configuration is essential** - Manual topic bridging required
4. **PID tuning matters** - High gains cause instability
5. **Direct Float64 commands work better** than trajectory commands for simple control

---

## 🚀 Next Steps / Improvements

1. ✅ **Implement keyboard control** using the working Float64 approach (**COMPLETED**)
2. ✅ **Add camera feed integration** with ros_gz_image bridge (**COMPLETED**)
3. 🎯 **Create object tracking node** using camera data (**IN PROGRESS**)
4. **Add trajectory smoothing** for better movement
5. **Implement joint limits enforcement** in software
6. **Add velocity control** for more natural movement

---

## 🎥 Camera Integration Guide

### ✅ Camera Setup (Already Configured)

Your camera system is already integrated:

1. **Camera Hardware**: Defined in `description/camera.xacro`
   - Attached to `tilt_platform` (moves with pan/tilt)
   - Gazebo sensor publishes to `/camera/image_raw`
   - Image size: 640x480, 10Hz update rate

2. **ROS2-Gazebo Bridge**: In `launch_sim_arti.launch.py`
   ```python
   ros_gz_image_bridge = Node(
       package="ros_gz_image",
       executable="image_bridge", 
       arguments=["/camera/image_raw"]
   )
   ```

### 🎬 How to View Camera Feed

#### Method 1: Using rqt_image_view (Recommended)
```bash
# In new terminal after launching simulation:
cd /home/tim/dev_ws
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
# Select '/camera/image_raw' from dropdown
```

#### Method 2: Check camera topics
```bash
# List available camera topics
ros2 topic list | grep camera

# Check camera info
ros2 topic echo /camera/image_raw --once

# Monitor camera stream info
python3 src/my_bot/camera_viewer.py
```

## � Camera System (✅ **FULLY WORKING**)

### Camera Hardware Integration
- **File**: `description/camera.xacro`
- **Mounting**: Attached to tilt platform (moves with pan/tilt)
- **Specifications**:
  - Resolution: 640x480 pixels
  - Frame Rate: 10 FPS  
  - Field of View: 62.4° (1.089 radians)
  - Color Format: RGB

### Camera Topics
- **Image Feed**: `/camera/image_raw` (sensor_msgs/Image)
- **Camera Info**: `/camera/camera_info` (sensor_msgs/CameraInfo)

### ROS2-Gazebo Camera Bridge
- **Bridge**: Configured in `launch_sim_arti.launch.py`
- **Function**: Converts Gazebo camera sensor data to ROS2 topics
```python
ros_gz_image_bridge = Node(
    package="ros_gz_image",
    executable="image_bridge",
    arguments=["/camera/image_raw"]
)
```

### Viewing Camera Feed
```bash
# Method 1: rqt_image_view (Recommended)
ros2 run rqt_image_view rqt_image_view
# Select '/camera/image_raw' from dropdown

# Method 2: Monitor camera data
python3 src/my_bot/camera_viewer.py

# Method 3: OpenCV display (if cv_bridge installed)
python3 src/my_bot/test_camera.py
```

---

## 🖥️ Quick Start Commands

### Launch Simulation
```bash
cd /home/tim/dev_ws
source install/setup.bash
ros2 launch my_bot launch_sim_arti.launch.py
```

### Control Options (Run in separate terminals)

#### 1. Keyboard Control
```bash
cd /home/tim/dev_ws && source install/setup.bash
python3 src/my_bot/keyboard_control_fixed.py
```

#### 2. Object Tracking (Auto-follow objects)
```bash
cd /home/tim/dev_ws && source install/setup.bash
python3 src/my_bot/object_tracker.py
```

#### 3. View Camera Feed
```bash
cd /home/tim/dev_ws && source install/setup.bash
ros2 run rqt_image_view rqt_image_view
# Select '/camera/image_raw' from dropdown
```

#### 4. Test Camera Connection
```bash
cd /home/tim/dev_ws && source install/setup.bash
python3 src/my_bot/camera_viewer.py
```

---

*Last Updated: Successfully achieved robot movement in Gazebo*
*Working Method: Direct Float64 commands via ROS2-Gazebo bridge*