# ROS2 Pan-Tilt Camera Control System Documentation

## 🎉 SUCCESS! The robot is now moving in Gazebo

This document explains how commands flow from ROS2 to Gazebo and key troubleshooting steps.

---

## 📊 Data Flow: ROS2 → Gazebo

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

## 🔄 Alternative Control Methods

### Method 1: Direct Float64 Commands (✅ Working)
- **Use**: `gentle_joint_commander.py`
- **Pros**: Simple, direct control
- **Cons**: No trajectory planning

### Method 2: Trajectory Controller (⚠️ Configured but not tested)
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

1. **Implement keyboard control** using the working Float64 approach
2. **Add camera feed integration** with ros_gz_image bridge
3. **Create object tracking node** using camera data
4. **Add trajectory smoothing** for better movement
5. **Implement joint limits enforcement** in software
6. **Add velocity control** for more natural movement

---

*Last Updated: Successfully achieved robot movement in Gazebo*
*Working Method: Direct Float64 commands via ROS2-Gazebo bridge*