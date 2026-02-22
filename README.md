# HERMES-percept

ROS 2 image gathering and perception nodes for the HERMES agent platform. The stack consists of two packages:

| Package | Node | Role |
|---|---|---|
| `hermes_percept_publisher` | `image_publisher` | Captures frames from a camera or file and publishes them on `camera/image_raw` |
| `hermes_percept_detection` | `image_detection` | Subscribes to `camera/image_raw`, runs CNN face detection, and publishes annotated images on `face_img` |

---

## Prerequisites

- **ROS 2** (tested on Jazzy / Ubuntu 24.04)
- **OpenCV 4** (`libopencv-dev`)
- **colcon** build tool
- Hardware: tested on a Raspberry Pi 5 running Ubuntu 24.04

Install OpenCV if needed:
```bash
sudo apt install libopencv-dev
```

---

## Building

The packages live at the root of this repository so it can be cloned directly into the `src/` directory of any ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/numbersletter/HERMES-percept.git
cd ~/ros2_ws
colcon build --symlink-install
```

colcon will find `hermes_percept_publisher` and `hermes_percept_detection` inside the cloned directory and build them automatically.

### Optional: ARM NEON SIMD optimizations

On ARM hardware (e.g. Raspberry Pi 5) you can enable NEON acceleration for the face-detection library:

```bash
colcon build --symlink-install --cmake-args -DENABLE_NEON=ON
```

---

## Running the nodes

Source the workspace after every build:

```bash
source ~/ros2_ws/install/setup.bash
```

### 1. Image Publisher node

Captures frames from a camera or media file and publishes them.

```bash
ros2 run hermes_percept_publisher image_publisher_node
```

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `source` | string | `"0"` | Camera index (`"0"`, `"1"`, …) or path to an image / video file |
| `publish_rate` | double | `15.0` | Publishing rate in Hz (range: `(0, 1000]`) |
| `flip_horizontal` | bool | `false` | Mirror the frame left-to-right |
| `flip_vertical` | bool | `false` | Flip the frame upside-down |
| `retry_on_failure` | bool | `false` | Attempt to re-open the capture source on read failure |

Pass parameters at launch with `--ros-args`:

```bash
# Use camera index 1 at 30 Hz with a horizontal mirror
ros2 run hermes_percept_publisher image_publisher_node \
  --ros-args -p source:=1 -p publish_rate:=30.0 -p flip_horizontal:=true

# Use a video file as the source
ros2 run hermes_percept_publisher image_publisher_node \
  --ros-args -p source:=/path/to/video.mp4
```

#### Published topics

| Topic | Type | Description |
|---|---|---|
| `camera/image_raw` | `sensor_msgs/msg/Image` | Raw captured frames (also available compressed via `image_transport`) |

---

### 2. Image Detection node

Subscribes to `camera/image_raw`, detects faces using a CNN model, draws bounding boxes and landmarks, then publishes annotated frames on `face_img`. Publishing is throttled to **1 Hz** (hardcoded) and only fires when at least one face is detected with a confidence score ≥ 80 (out of 100).

```bash
ros2 run hermes_percept_detection image_detection_node
```

#### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `camera/image_raw` | `sensor_msgs/msg/Image` | Input image stream |

#### Published topics

| Topic | Type | Description |
|---|---|---|
| `face_img` | `sensor_msgs/msg/Image` | Annotated image (bounding box + landmarks) when a face is detected |

#### Remap the input topic

```bash
ros2 run hermes_percept_detection image_detection_node \
  --ros-args --remap camera/image_raw:=/my_camera/image_raw
```

---

## Running both nodes together

Open two terminals (both sourced), or use a launch file:

**Terminal 1 – publisher:**
```bash
ros2 run hermes_percept_publisher image_publisher_node
```

**Terminal 2 – detector:**
```bash
ros2 run hermes_percept_detection image_detection_node
```

---

## Component containers

Both packages expose ROS 2 composable node components, allowing them to run in the same process for lower latency:

| Package | Component class |
|---|---|
| `hermes_percept_publisher` | `hermes_percept::ImagePublisherComponent` |
| `hermes_percept_detection` | `hermes_percept::ImageDetectionComponent` |

```bash
# Start a component manager
ros2 run rclcpp_components component_container

# Load the publisher (in a second terminal)
ros2 component load /ComponentManager hermes_percept_publisher hermes_percept::ImagePublisherComponent

# Load the detector (in a third terminal)
ros2 component load /ComponentManager hermes_percept_detection hermes_percept::ImageDetectionComponent
```

---

## License

Apache-2.0 — see [LICENSE](LICENSE).
