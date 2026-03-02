# Vision Subsystem

This document explains the vision system that detects AprilTags and provides pose estimation for the robot.

## Table of Contents

1. [Overview](#overview)
2. [AprilTags Explained](#apriltags-explained)
3. [Hardware Setup](#hardware-setup)
4. [PhotonVision Pipeline](#photonvision-pipeline)
5. [Pose Estimation](#pose-estimation)
6. [Code Structure](#code-structure)
7. [Performance Optimization](#performance-optimization)
8. [Calibration and Tuning](#calibration-and-tuning)

---

## Overview

The vision system enables the robot to know its position on the field by detecting AprilTags - special markers placed at known locations around the field.

```
    Field with AprilTags

    ┌──────────────────────────────────────────┐
    │ [1]                              [2]     │
    │                                          │
    │      ┌───┐                               │
    │ [3]  │ R │  Robot sees tags [1] and [4]  │
    │      └───┘  → Calculates position        │
    │                                          │
    │ [4]                              [5]     │
    └──────────────────────────────────────────┘

    Tags have known field positions (from game manual)
```

### Why Vision Matters

- **Autonomous accuracy**: Know exactly where to start and drive
- **Odometry correction**: Fix drift accumulated during match
- **Target tracking**: Find game elements or scoring locations
- **Defense avoidance**: Know where other robots are (with additional processing)

---

## AprilTags Explained

### What is an AprilTag?

An AprilTag is a 2D barcode specifically designed for robotics:

```
    ┌─────────────────┐
    │ ██░░██░░██░░██ │
    │ ░░██░░██░░██░░ │
    │ ██░░░░░░░░██░░ │
    │ ░░██░░██░░██░░ │
    │ ██░░██░░░░░░██ │
    │ ░░██░░██░░██░░ │
    │ ██░░██░░██░░██ │
    └─────────────────┘
         Tag ID: 7
```

### Properties

- **Unique ID**: Each tag has a number (0-586 in 36h11 family)
- **Orientation**: The pattern encodes which way is "up"
- **Known size**: Physical dimensions are standardized (6" for FRC)
- **High contrast**: Black and white for reliable detection

### How Detection Works

1. Camera captures image
2. Find square regions with black border
3. Decode inner pattern to get ID
4. Estimate pose using known tag size

### Pose from a Single Tag

From one tag detection, we can determine:
- **Distance**: From apparent tag size
- **Angle**: From tag position in frame
- **Rotation**: From tag orientation

```
    Camera View

    ┌─────────────────────────┐
    │                         │
    │        ┌─────┐          │
    │        │ Tag │ ← Small = far
    │        └─────┘   Offset = angle
    │                         │
    └─────────────────────────┘
```

### Multi-Tag Pose Estimation

When multiple tags are visible, we get more accurate poses:
- Cross-reference distances
- Resolve ambiguities
- Reduce noise through averaging

---

## Hardware Setup

### Camera Configuration

We use **4 OV2311 PhotonVision cameras** mounted on the rear of the robot, all angled upward to see AprilTags on the field:

```
                   Front
    ┌─────────────────────────────┐
    │                             │
    │                             │
    │                             │
    │    RL ◤             ◥ RR    │   FR = Front-Right (rear-mounted, looking back-right)
    │       ╲             ╱       │   FL = Front-Left (rear-mounted, looking back-left)
    │        ╲           ╱        │   RR = Rear-Right (rear-mounted, looking further back-right)
    │    FL ◤ ╲         ╱ ◥ FR    │   RL = Rear-Left (rear-mounted, looking further back-left)
    └─────────────────────────────┘
                   Back
          (all cameras face rearward)
```

### Why Multiple Cameras?

- **Wide rear coverage**: All cameras face rearward with overlapping fields of view
- **Redundancy**: If one camera is blocked, others still work
- **Multi-tag**: More cameras = more simultaneous tag detections

### Camera Specifications

OV2311 cameras:
- **Resolution**: 1600×1200
- **Frame rate**: Up to 60 FPS
- **Field of view**: ~80°
- **Global shutter**: Prevents motion blur

### Camera Mounting

Each camera position is precisely measured using quaternions for rotation. The cameras are mounted at the rear of the robot, pitched upward to see tags:

```java
// Front-Right camera: rear-mounted, pitched 20° up, yawed 55° right
robotToFrontRightCamera = new Transform3d(
    Inches.of(-10.572),   // Behind center
    Inches.of(-12.337),   // Right of center
    Inches.of(16.688),    // Up from floor
    new Rotation3d(new Quaternion(0.8735, -0.0802, -0.1540, -0.4547))
);

// Rear-Right camera: further back, pitched 15° up, yawed 135° right
robotToBackRightCamera = new Transform3d(
    Inches.of(-13.1623),  // Further behind center
    Inches.of(-12.1623),  // Right of center
    Inches.of(20.26674),  // Higher up
    new Rotation3d(new Quaternion(-0.3794, 0.1206, 0.0500, 0.9160))
);
```

Accurate measurements are **critical** - errors here directly cause pose errors.

---

## PhotonVision Pipeline

### What is PhotonVision?

PhotonVision is open-source vision software that runs on a coprocessor (Orange Pi, Raspberry Pi, etc.):

```
    Camera → Coprocessor → NetworkTables → RoboRIO
              (PhotonVision)              (Robot Code)
```

### Processing Pipeline

```
┌─────────────┐
│ Raw Image   │
└──────┬──────┘
       ▼
┌─────────────┐
│ Thresholding│  Convert to black/white
└──────┬──────┘
       ▼
┌─────────────┐
│ Contour     │  Find tag boundaries
│ Detection   │
└──────┬──────┘
       ▼
┌─────────────┐
│ Decoding    │  Read tag ID from pattern
└──────┬──────┘
       ▼
┌─────────────┐
│ Pose        │  Calculate camera-to-tag
│ Estimation  │  transform
└──────┬──────┘
       ▼
┌─────────────┐
│ Multi-Tag   │  Combine multiple tags
│ Fusion      │  for better estimate
└──────┬──────┘
       ▼
┌─────────────┐
│ Output to   │  Publish results
│ NetworkTables│
└─────────────┘
```

### Pipeline Output

For each camera frame, PhotonVision publishes:
- List of detected tags (ID, corners, pose)
- Best estimated robot pose
- Timestamp of capture
- Ambiguity metric (how confident)

---

## Pose Estimation

### From Camera Pose to Robot Pose

```
robot_pose = camera_pose × inverse(robot_to_camera)

Where:
- camera_pose = where camera is on field (from AprilTag)
- robot_to_camera = where camera is on robot (measured)
```

### Handling Multiple Observations

Each camera might see different tags at different times. We need to:

1. **Collect all observations** from all cameras
2. **Assign confidence** based on distance, ambiguity, tag count
3. **Fuse with odometry** using Kalman filter

### Sigmoid-Based Pose Scoring

Before fusing vision observations with odometry, each observation is scored using a series of tests. Each test uses a **normalized sigmoid function** to produce a score between 0 and 1:

```java
// Sigmoid function with tunable midpoint and steepness
double normalizedSigmoid(double x, double midpoint, double steepness) {
    return 1.0 / (1.0 + Math.exp(-steepness * (x - midpoint)));
}
```

The scoring tests include:

| Test | What it Checks | Penalty Condition |
|------|---------------|-------------------|
| `unambiguous` | Single-tag pose ambiguity ratio | Ambiguity > 0.15 |
| `pitchError` | Robot pitch (should be ~0) | Pose shows robot tilted forward/back |
| `rollError` | Robot roll (should be ~0) | Pose shows robot tilted sideways |
| `heightError` | Robot elevation (should be ~0) | Pose shows robot floating/underground |
| `withinBoundaries` | Position inside field | Pose outside field boundaries |
| `moreThanZeroTags` | At least one tag visible | No tags detected |
| `distanceToTags` | Proximity to detected tags | Tags far away (>4m) |

The final score is the product of all individual test scores. Low-scoring observations are rejected or given higher standard deviations, preventing bad measurements from corrupting the pose estimate.

### Standard Deviation Calculation

We trust observations less when:
- Tags are far away (larger measurement error)
- Only one tag visible (pose ambiguity)
- High reprojection error (bad fit)

```java
// Example: Standard deviation increases with distance
double baseStdDev = 0.5;  // meters at 1m distance
double distanceMultiplier = avgDistance / 1.0;
double stdDev = baseStdDev * distanceMultiplier;

// Trust multi-tag more than single-tag
if (tagCount > 1) {
    stdDev *= 0.5;  // Half the uncertainty
}
```

### Kalman Filter Fusion

The drive subsystem maintains a pose estimator that combines:

```
┌─────────────────┐
│ Wheel Odometry  │  Fast updates (50Hz)
│ High drift      │  Low noise per update
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Kalman Filter   │  Optimal fusion
│ (Pose Estimator)│  based on uncertainties
└────────┬────────┘
         ▲
         │
┌────────┴────────┐
│ Vision Updates  │  Slower updates (10-30Hz)
│ No drift        │  Higher noise per update
└─────────────────┘
```

Result: Best of both worlds - responsive AND accurate.

---

## Code Structure

### Files

```
subsystems/vision/
├── Vision.java             # Main subsystem
├── VisionConstants.java    # Camera positions, thresholds
├── VisionIO.java          # Hardware interface
├── VisionIOPhotonVision.java  # Real camera implementation
└── VisionIOSim.java       # Simulation implementation
```

### VisionIO Interface

```java
public interface VisionIO {
    /** Updates the set of inputs. */
    void updateInputs(VisionIOInputs inputs);

    class VisionIOInputs {
        // Per-camera observations
        public PoseObservation[] observations = new PoseObservation[0];
    }

    record PoseObservation(
        Pose3d pose,           // Estimated robot pose
        double timestamp,      // When image was captured
        double ambiguity,      // 0 = certain, 1 = ambiguous
        int tagCount,          // Number of tags seen
        double avgDistance     // Average distance to tags
    ) {}
}
```

### Vision.java Key Methods

```java
public class Vision extends SubsystemBase {

    @Override
    public void periodic() {
        // Update inputs from all cameras
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
        }

        // Process each observation
        for (var observation : getAllObservations()) {
            // Skip if too ambiguous
            if (observation.ambiguity() > maxAmbiguity) continue;

            // Calculate standard deviations
            var stdDevs = calculateStdDevs(observation);

            // Send to drive subsystem
            drive.addVisionMeasurement(
                observation.pose().toPose2d(),
                observation.timestamp(),
                stdDevs
            );
        }
    }

    private Matrix<N3, N1> calculateStdDevs(PoseObservation obs) {
        double xyStdDev = baseXYStdDev * obs.avgDistance();
        double thetaStdDev = baseThetaStdDev * obs.avgDistance();

        // Trust multi-tag more
        if (obs.tagCount() > 1) {
            xyStdDev *= 0.5;
            thetaStdDev *= 0.5;
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
}
```

### Integration with Drive

```java
// In Drive.java
public void addVisionMeasurement(
    Pose2d visionPose,
    double timestamp,
    Matrix<N3, N1> stdDevs) {

    poseEstimator.addVisionMeasurement(
        visionPose,
        timestamp,
        stdDevs
    );
}
```

---

## Performance Optimization

### The Problem

Vision processing can be slow and block the main robot loop:
- NetworkTables reads can take 1-5ms
- Processing 4 cameras = potential 20ms delay
- Main loop budget is only 20ms total

### Solution: Background Thread

We run vision I/O in a separate thread:

```java
public class VisionIOPhotonVision implements VisionIO {
    private final Thread updateThread;
    private volatile PoseObservation[] latestObservations;

    public VisionIOPhotonVision() {
        updateThread = new Thread(this::pollCameras);
        updateThread.setDaemon(true);
        updateThread.start();
    }

    private void pollCameras() {
        while (true) {
            // Read from all cameras (blocking is OK here)
            var observations = readFromNetworkTables();

            // Atomically update cached value
            latestObservations = observations;

            // Sleep to target ~50Hz
            Thread.sleep(20);
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Non-blocking - just copy cached value
        inputs.observations = latestObservations;
    }
}
```

### Timestamp Handling

Vision observations have latency:
1. Image captured (timestamp T)
2. Processing (takes ~30-50ms)
3. Arrives at RoboRIO (at time T + latency)

We use the **capture timestamp**, not arrival time, for pose fusion. The Kalman filter handles the historical update correctly.

---

## Calibration and Tuning

### Camera Calibration

Cameras have lens distortion that must be corrected:

```
    Uncalibrated              Calibrated
    ┌─────────────┐          ┌─────────────┐
    │  )     (    │          │  |     |    │
    │  )     (    │    →     │  |     |    │
    │  )     (    │          │  |     |    │
    └─────────────┘          └─────────────┘
    (barrel distortion)      (straight lines)
```

PhotonVision has built-in calibration:
1. Print checkerboard pattern
2. Hold in front of camera at various angles
3. PhotonVision calculates correction matrix

### Camera Position Calibration

To verify camera positions:
1. Place robot at known field position
2. Check reported pose matches actual pose
3. Adjust camera transforms if needed

### Standard Deviation Tuning

If poses are noisy or jumpy:
- **Increase** vision std devs (trust odometry more)
- **Decrease** odometry std devs

If poses drift over time:
- **Decrease** vision std devs (trust vision more)
- **Increase** odometry std devs

### Ambiguity Threshold

The `maxAmbiguity` parameter filters unreliable detections:
- **Lower** (0.1-0.2): Only use very confident detections
- **Higher** (0.3-0.5): Accept more detections, may be noisier

### Debugging Tips

1. **Check timestamps**: Are observations arriving with reasonable latency?
2. **Visualize poses**: Plot both odometry and vision poses
3. **Check tag visibility**: Are expected tags being detected?
4. **Verify camera transforms**: Does pose jump when rotating in place?

---

## Related Documentation

- [OVERVIEW.md](OVERVIEW.md) - Project architecture
- [DRIVE.md](DRIVE.md) - How vision integrates with odometry
- [PhotonVision Docs](https://docs.photonvision.org/) - Official documentation
