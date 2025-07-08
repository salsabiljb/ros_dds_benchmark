# ros_dds_benchmark

## ROS 2 Camera Benchmarking with DDS

This project benchmarks image stream performance in ROS 2 using different DDS backends (Fast DDS and Cyclone DDS) and QoS policies. The benchmark includes recording reception timestamps, computing latency, tracking frame drops, and comparing results across different configurations.

---

##  1. Playback Setup

### Goal

Replay a rosbag with multiple image topics and monitor:

* Header timestamp
* Reception time
* Frame ID & Sequence
* Latency (calculated)

### Image Subscriber Node

Implemented in **C++**:
File: `src/image_subscriber_node.cpp`

Features:

* Auto-discovers image topics.
* Logs relevant metadata per frame.
* Creates a CSV file for each topic under `logs/`. (after each run before deleting logs I save them into seperate folders namely : trial_1_cyclone , trial1_fastrtps, default_dds_t1_logs)

### Run Instructions

**Build and source the package**:

```bash
colcon build --symlink-install
source install/ros2_dds_camera_benchmark/share/ros2_dds_camera_benchmark/local_setup.bash

```

**Play the bag**:

```bash
ros2 bag play <bag_file>
In my case: ros2 bag play ~/ros_dds_native/rosbags/r2b_storage 
```

**Run the subscriber node**:

```bash
ros2 run ros2_dds_camera_benchmark image_subscriber_node

```

*Optional:* if using simulated time like i did due to some issues with msg time and sys clock:

```bash
ros2 bag play ~/ros_dds_native/rosbags/r2b_storage --clock

ros2 run ros2_dds_camera_benchmark image_subscriber_node --ros-args --param use_sim_time:=true
```

---

##  2. Log Analysis

### Script: `scripts/analyze_logs.py`

* Parses each CSV log.
* Calculates latency statistics.
* Plots latency series and histograms.

### Output

* Summary printed in terminal.
* Plots saved in `performance_report/` after multiple runs I organized them a bit each in folder for both DDS.

### Run Analysis:

```bash
python3 scripts/analyze_logs.py logs/*.csv
Note since I use native env I also needed a seperate env for python
```

---

## 3. DDS Comparison

tested both **Fast DDS** and **Cyclone DDS**:

**Switch DDS Implementation**:

```bash
# Fast DDS (default)
unset RMW_IMPLEMENTATION

# Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Each run logs separate performance files, and latency differences were observed between DDS backends (as analyzed using the logs and performance plots).

---

## 4. QoS Configuration (Not Performed Yet)

**Planned but not executed** (TO DO!)

### What to Evaluate:

* **Reliability**: `BEST_EFFORT` vs `RELIABLE`
* **History**: `KEEP_LAST` with depth = 1, 5, 10
* **Intra-process Communication**

### XML Profile Setup

Create `config/qos_profiles.xml`:

```xml
<dds>
  <profiles>
    <qos_profile name="custom_qos_profile" is_default="true">
      <reliability>
        <kind>BEST_EFFORT</kind>
      </reliability>
      <history>
        <kind>KEEP_LAST</kind>
        <depth>5</depth>
      </history>
    </qos_profile>
  </profiles>
</dds>
```

Then run:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/config/qos_profiles.xml
```

Use the same `ros2 run` and `ros2 bag play` commands to benchmark.

---

## 5. Useful Commands

| Command                                                    | Purpose                          |                                      |
| ---------------------------------------------------------- | -------------------------------- | ------------------------------------ |
| `ros2 bag play <file>`                                     | Replay ROS bag                   |                                      |
| `ros2 run ros2_dds_camera_benchmark image_subscriber_node` | Run the image subscriber         |                                      |
| `python3 scripts/analyze_logs.py logs/*.csv`               | Analyze latency logs             |                                      |
| \`top -p \$(pgrep -f image\_subscriber\_node               | head -n 1)\`                     | Monitor CPU usage of subscriber node |
| `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`             | Switch to Cyclone DDS            |                                      |
| `unset RMW_IMPLEMENTATION`                                 | Reset to default DDS (Fast DDS)  |                                      |
| `export FASTRTPS_DEFAULT_PROFILES_FILE=...`                | Set QoS XML profile for Fast DDS |                                      |


