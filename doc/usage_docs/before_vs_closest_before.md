# Before vs ClosestBefore: Use Case Comparison

This document provides a detailed comparison of `follower::Before` and `follower::ClosestBefore` to help you choose the right captor for your robotics application.

---

## Table of Contents

1. [Quick Comparison](#quick-comparison)
2. [Core Differences](#core-differences)
3. [Robotics Use Cases](#robotics-use-cases)
4. [Decision Flowchart](#decision-flowchart)
5. [Code Examples](#code-examples)
6. [Performance Considerations](#performance-considerations)
7. [Common Mistakes](#common-mistakes)

---

## Quick Comparison

| Aspect | `Before` | `ClosestBefore` |
|--------|----------|-----------------|
| **Output** | All elements before boundary | Single element closest to boundary |
| **Count** | 0 to N elements | Exactly 0 or 1 element |
| **Boundary** | `upper_stamp - delay` | `lower_stamp - delay` |
| **Staleness Check** | No (takes any data) | Yes (rejects if outside `period`) |
| **Fails if** | No proof element exists | Data too stale OR gap in window |
| **Parameters** | `delay` | `period`, `delay` |

---

## Core Differences

### Difference 1: Quantity of Data

```
Sensor data: [60, 70, 80, 85, 90, 95, 98, 102, 110]
Driver at t=100

BEFORE (boundary = 100):
┌─────────────────────────────────────────────────────┐
│  Captures: [60, 70, 80, 85, 90, 95, 98]             │
│  Count: 7 elements                                   │
│  "Everything that happened before"                   │
└─────────────────────────────────────────────────────┘

CLOSEST_BEFORE (boundary = 100, period = 20):
┌─────────────────────────────────────────────────────┐
│  Window: [80, 100)                                   │
│  Captures: [98]  (closest to 100 within window)     │
│  Count: 1 element                                    │
│  "Most recent state"                                 │
└─────────────────────────────────────────────────────┘
```

### Difference 2: Data Freshness Enforcement

```
Sensor data: [10, 20, 30]  ← All old data!
Driver at t=100

BEFORE:
┌─────────────────────────────────────────────────────┐
│  boundary = 100                                      │
│  Captures: [10, 20, 30]  ← Takes stale data!        │
│  State: PRIMED (if proof element exists)            │
│                                                      │
│  ⚠️ No freshness check - accepts ANY old data       │
└─────────────────────────────────────────────────────┘

CLOSEST_BEFORE (period = 20):
┌─────────────────────────────────────────────────────┐
│  boundary = 100                                      │
│  Window: [80, 100)                                   │
│  Data in window: NONE (30 < 80)                     │
│  State: ABORT                                        │
│                                                      │
│  ✓ Rejects stale data - enforces freshness          │
└─────────────────────────────────────────────────────┘
```

### Difference 3: Failure Modes

```
BEFORE fails (RETRY) when:
├─ Queue is empty
└─ No "proof" element >= boundary exists
   (Cannot prove all data has arrived)

CLOSEST_BEFORE fails when:
├─ RETRY: No data in queue at all
├─ ABORT: Oldest element is >= boundary (gap in data)
└─ ABORT: No element in [boundary-period, boundary) window
```

---

## Robotics Use Cases

### Use Case 1: IMU Integration (Use `Before`)

**Scenario:** Visual-Inertial Odometry needs ALL IMU readings between camera frames.

```
Camera: 30fps → frames at t=0, 33, 66, 100, 133...
IMU: 200Hz → readings at t=0, 5, 10, 15, 20, 25, 30, 33, 38...

Between camera frames at t=66 and t=100:
IMU readings: [66, 71, 76, 81, 86, 91, 96]

VIO Algorithm needs to INTEGRATE all 7 readings:
    delta_rotation = ∫ gyro dt
    delta_velocity = ∫ accel dt
    delta_position = ∫∫ accel dt²
```

**Why `Before`:**
- Need ALL data, not just one
- Missing any IMU reading corrupts the integration
- Output is variable-length (depends on timing)

```cpp
driver::Next<CameraFrame> camera;
follower::Before<ImuReading> imu{0};

// Each camera frame captures ~6-7 IMU readings
// [imu_66, imu_71, imu_76, imu_81, imu_86, imu_91, imu_96]
```

---

### Use Case 2: Transform Lookup (Use `ClosestBefore`)

**Scenario:** Need robot pose when camera image was captured.

```
Camera: 30fps → image at t=100
Odometry: 100Hz → poses at t=90, 100, 110...

Question: "Where was the robot at t=100?"
Answer: Use pose closest to t=100
```

**Why `ClosestBefore`:**
- Need exactly ONE pose
- Want the MOST RECENT (closest) pose
- Stale pose is dangerous (robot moved!)

```cpp
driver::Next<CameraFrame> camera;
follower::ClosestBefore<RobotPose> pose{15ms, 0};
// period=15ms: pose must be within 15ms of image
// If pose is older, ABORT (safety!)

// Captures: single pose at t=98 or t=100
```

---

### Use Case 3: Event Logging (Use `Before`)

**Scenario:** Log all warnings/errors that occurred before a robot action.

```
Robot action at t=1000
Warning events: [800, 850, 920, 980]

Question: "What warnings happened before the action?"
Answer: All of them: [800, 850, 920, 980]
```

**Why `Before`:**
- Need complete history
- Variable number of events
- No event is "more important" than another

```cpp
driver::Next<RobotAction> actions;
follower::Before<WarningEvent> warnings{0};

// Captures ALL warnings before the action
// Could be 0, could be 100 - depends on what happened
```

---

### Use Case 4: Lidar-Camera Calibration (Use `ClosestBefore`)

**Scenario:** For each camera frame, get the corresponding LiDAR scan.

```
Camera: 30fps → frames at t=100, 133, 166...
LiDAR: 10fps → scans at t=50, 150, 250...

For camera at t=100:
    Closest LiDAR: t=50 (but 50ms old!)
    
For camera at t=166:
    Closest LiDAR: t=150 (16ms old - acceptable)
```

**Why `ClosestBefore`:**
- Need single scan to match with single frame
- Freshness matters (stale scan = wrong geometry)
- Period parameter rejects too-old data

```cpp
driver::Next<CameraFrame> camera;
follower::ClosestBefore<LidarScan> lidar{50ms, 0};
// period=50ms: scan must be within 50ms

// t=100: Would ABORT (scan 50 is 50ms old, edge case)
// t=166: Captures scan at t=150 (16ms old - OK)
```

---

### Use Case 5: Sensor Diagnostics (Use `Before`)

**Scenario:** Collect all sensor health messages for monitoring dashboard.

```
Dashboard update at t=1000
Health messages: [920: "temp=45C", 950: "voltage=12.1V", 980: "current=2.3A"]

Dashboard needs: ALL health metrics, not just one
```

**Why `Before`:**
- Multiple independent metrics
- Need complete picture
- No single "closest" metric makes sense

```cpp
driver::Throttled<DashboardTick> dashboard{1000ms};
follower::Before<HealthMessage> health{0};

// Captures all health messages since last update
```

---

### Use Case 6: Wheel Odometry for Motion Model (Use `ClosestBefore`)

**Scenario:** Localization particle filter needs odometry delta.

```
Localization runs at: 20Hz → t=50, 100, 150...
Wheel odometry at: 50Hz → t=40, 60, 80, 100, 120...

For localization at t=100:
    Need: single odometry reading to compute delta
    Best: reading at t=100 (exact match!)
    Acceptable: reading at t=80 (20ms old)
```

**Why `ClosestBefore`:**
- Need single pose to compute delta from last
- Stale odometry = wrong motion model
- Period rejects unacceptably old data

```cpp
driver::Next<LocalizationTick> localization;
follower::ClosestBefore<WheelOdom> odom{25ms, 0};
// Must have odometry within 25ms
```

---

## Decision Flowchart

```
                    ┌─────────────────────────┐
                    │ How many elements do    │
                    │ you need from this      │
                    │ follower stream?        │
                    └───────────┬─────────────┘
                                │
                ┌───────────────┴───────────────┐
                │                               │
                ▼                               ▼
        ┌───────────────┐               ┌───────────────┐
        │  ALL elements │               │ ONE element   │
        │  (variable N) │               │ (exactly 1)   │
        └───────┬───────┘               └───────┬───────┘
                │                               │
                ▼                               ▼
        ┌───────────────┐               ┌───────────────────┐
        │ Does data     │               │ Does freshness    │
        │ freshness     │               │ matter?           │
        │ matter?       │               │ (reject stale)    │
        └───────┬───────┘               └─────────┬─────────┘
                │                                 │
        ┌───────┴───────┐                 ┌───────┴───────┐
        │               │                 │               │
        ▼               ▼                 ▼               ▼
    ┌───────┐       ┌───────┐         ┌───────┐       ┌───────┐
    │  Yes  │       │  No   │         │  Yes  │       │  No   │
    └───┬───┘       └───┬───┘         └───┬───┘       └───┬───┘
        │               │                 │               │
        ▼               ▼                 ▼               ▼
   Consider         ┌────────┐      ┌─────────────┐   Consider
   CountBefore      │ BEFORE │      │CLOSEST_BEFORE│   Latched
   or custom        └────────┘      └─────────────┘   
```

---

## Code Examples

### Example 1: Visual-Inertial Odometry

```cpp
#include <flow/flow.hpp>

// Camera drives the pipeline
driver::Next<Dispatch<int64_t, CameraFrame>, NoLock> camera;

// Need ALL IMU readings for preintegration
follower::Before<Dispatch<int64_t, ImuReading>, NoLock> imu{0};

// Need SINGLE pose at frame capture time
follower::ClosestBefore<Dispatch<int64_t, RobotPose>, NoLock> pose{
    20,  // period: pose must be within 20ms
    0    // delay: no offset
};

Synchronizer sync{camera, imu, pose};

void process() {
    std::vector<Dispatch<int64_t, CameraFrame>> cam_data;
    std::vector<Dispatch<int64_t, ImuReading>> imu_data;    // Multiple!
    std::vector<Dispatch<int64_t, RobotPose>> pose_data;    // Single!
    
    State state = sync.capture(
        std::back_inserter(cam_data),
        std::back_inserter(imu_data),
        std::back_inserter(pose_data)
    );
    
    if (state == State::PRIMED) {
        // cam_data: 1 frame
        // imu_data: ~6-7 IMU readings (variable!)
        // pose_data: 1 pose
        
        // Integrate IMU
        ImuDelta delta = preintegrate(imu_data);
        
        // Run VIO with frame, IMU delta, and pose prior
        run_vio(cam_data[0], delta, pose_data[0]);
    }
}
```

### Example 2: Multi-Sensor Logging

```cpp
// Log entry triggers capture
driver::Throttled<Dispatch<int64_t, LogTrigger>, NoLock> logger{1000};  // 1Hz

// Collect ALL events since last log
follower::Before<Dispatch<int64_t, SensorEvent>, NoLock> events{0};

// Get LATEST system status
follower::ClosestBefore<Dispatch<int64_t, SystemStatus>, NoLock> status{
    500,  // Must have status within 500ms
    0
};

void log_iteration() {
    auto [trigger, events, status] = capture();
    
    // events: could be 0, 10, or 100 events
    // status: exactly 1 status reading
    
    write_log_entry(trigger.stamp, events, status);
}
```

---

## Performance Considerations

### Memory Usage

```
Before:
├─ Stores variable number of elements
├─ Memory: O(N) where N = data rate × time between captures
└─ Example: 200Hz IMU, 30Hz camera → ~7 elements/capture

ClosestBefore:
├─ Stores exactly 1 element
├─ Memory: O(1)
└─ Always predictable
```

### Processing Time

```
Before:
├─ Must iterate to find all elements < boundary
├─ Processing: O(N) per capture
└─ Must process variable-length output

ClosestBefore:
├─ Iterates to find single closest element
├─ Processing: O(N) to find, but O(1) output
└─ Fixed processing for output
```

### When Performance Matters

```cpp
// High-rate system: 1000Hz driver, 10000Hz follower

// BEFORE: Captures ~10 elements per iteration
// 1000 iterations/sec × 10 elements = 10,000 elements/sec processed

// CLOSEST_BEFORE: Captures 1 element per iteration
// 1000 iterations/sec × 1 element = 1,000 elements/sec processed

// If you only NEED the latest value, ClosestBefore is 10x more efficient!
```

---

## Common Mistakes

### Mistake 1: Using Before When You Only Need One Value

```cpp
// WRONG: Captures all poses, then only uses last one
follower::Before<RobotPose> pose{0};
auto poses = capture_poses();
use_pose(poses.back());  // Wasted effort getting all others!

// RIGHT: Directly get the one you need
follower::ClosestBefore<RobotPose> pose{20, 0};
auto pose = capture_pose();  // Just one!
use_pose(pose);
```

### Mistake 2: Using ClosestBefore When You Need History

```cpp
// WRONG: Only gets one IMU, integration is incorrect!
follower::ClosestBefore<ImuReading> imu{10, 0};
auto imu = capture_imu();  // Missing 6 other readings!
delta = integrate(imu);    // Huge error!

// RIGHT: Get all IMU readings for proper integration
follower::Before<ImuReading> imu{0};
auto imus = capture_imus();  // All 7 readings
delta = integrate(imus);     // Correct!
```

### Mistake 3: Ignoring Staleness Requirements

```cpp
// WRONG: Accepts arbitrarily old pose data
follower::Before<RobotPose> pose{0};
// Could return pose from 10 seconds ago if stream stalled!

// RIGHT: Fail if pose is too old
follower::ClosestBefore<RobotPose> pose{100, 0};  // Max 100ms old
// ABORTs if pose older than 100ms - safer!
```

### Mistake 4: Period Too Small in ClosestBefore

```cpp
// WRONG: Period smaller than data interval
// Data arrives every 10ms, but period is 5ms
follower::ClosestBefore<Data> data{5, 0};  // Window [boundary-5, boundary)
// Frequently ABORTs because window misses data!

// RIGHT: Period >= data interval
follower::ClosestBefore<Data> data{15, 0};  // Window [boundary-15, boundary)
// Reliably captures data
```

---

## Summary

| Question | Answer |
|----------|--------|
| Need ALL historical data? | Use `Before` |
| Need just the LATEST value? | Use `ClosestBefore` |
| Data freshness critical? | Use `ClosestBefore` (has period check) |
| Integrating over time? | Use `Before` |
| Looking up state at a moment? | Use `ClosestBefore` |
| Variable output size OK? | Use `Before` |
| Need predictable output size? | Use `ClosestBefore` |
| Don't care about stale data? | Use `Before` |
| Stale data is dangerous? | Use `ClosestBefore` |
