# Combined Driver-Follower Workflows

This document demonstrates real-world synchronization workflows combining various driver and follower types.

---

## Table of Contents

1. [Workflow 1: Frame-by-Frame Sensor Fusion](#workflow-1-frame-by-frame-sensor-fusion)
2. [Workflow 2: Sliding Window Analysis](#workflow-2-sliding-window-analysis)
3. [Workflow 3: Rate-Limited Processing with State](#workflow-3-rate-limited-processing-with-state)
4. [Workflow 4: Exact Timestamp Synchronization](#workflow-4-exact-timestamp-synchronization)
5. [Workflow 5: Batch Processing with Optional Streams](#workflow-5-batch-processing-with-optional-streams)
6. [Workflow 6: Interpolation with Ranged Data](#workflow-6-interpolation-with-ranged-data)

---

## Workflow 1: Frame-by-Frame Sensor Fusion

**Scenario:** Fuse camera frames with IMU data and robot pose

**Components:**
- `driver::Next` - Camera frames (driving stream)
- `follower::ClosestBefore` - IMU readings (high frequency)
- `follower::Latched` - Robot pose (slow updates)

### Setup

```cpp
using Stamp = int;  // milliseconds

// Camera at 30fps (33ms period)
driver::Next<Dispatch<Stamp, CameraFrame>, NoLock> camera;

// IMU at 100Hz (10ms period)
follower::ClosestBefore<Dispatch<Stamp, ImuData>, NoLock> imu{
    15,  // period: slightly larger than IMU period
    0    // delay: no delay
};

// Pose updates at ~1Hz
follower::Latched<Dispatch<Stamp, RobotPose>, NoLock> pose{
    500  // min_period: expect updates every ~500ms or slower
};
```

### Execution Trace

```
TIME: t=1000ms

DATA STATE:
    Camera queue: [967, 1000, 1033]
    IMU queue:    [970, 980, 990, 1000, 1010, 1020]
    Pose queue:   [500, 1000]
    Pose latched: Dispatch{500, pose_A}

SYNCHRONIZATION:
    
    STEP 1: Driver (Camera) establishes range
    ├─ camera.locate()
    │   ├─ oldest_stamp() = 967
    │   └─ range = {967, 967}
    │
    └─ State: PRIMED

    STEP 2: Follower 1 (IMU) - ClosestBefore
    ├─ imu.locate(range={967, 967})
    │   ├─ boundary = 967 - 0 = 967
    │   ├─ window = [967-15, 967) = [952, 967)
    │   │
    │   ├─ Find closest in window:
    │   │   └─ No IMU stamp in [952, 967)
    │   │      (closest before is 990, which is > 967!)
    │   │
    │   └─ Wait... this would ABORT!
    │
    │   ⚠️ PROBLEM: IMU data is AHEAD of camera!
    │      Camera stamp 967 is older than IMU stamps.
    │
    └─ State: ABORT

    → Frame dropped due to IMU data gap!

NEXT SYNCHRONIZATION (after more data):

    Camera queue: [1000, 1033, 1067]
    IMU queue:    [980, 990, 1000, 1010, 1020, 1030]
    
    STEP 1: Driver range = {1000, 1000}
    
    STEP 2: IMU.locate(range={1000, 1000})
    ├─ boundary = 1000, window = [985, 1000)
    ├─ Find closest: stamp 990 is in [985, 1000) ✓
    └─ State: PRIMED

    STEP 3: Pose.locate(range={1000, 1000})
    ├─ boundary = 1000 - 500 = 500
    ├─ Find stamp <= 500: stamp 500 ✓
    │   OR use latched value
    └─ State: PRIMED

    EXTRACTION:
    ├─ Camera: Dispatch{1000, frame}
    ├─ IMU:    Dispatch{990, imu_reading}
    └─ Pose:   Dispatch{500, pose_A} (latched)

    RESULT: State::PRIMED
    
    Synchronized output for t=1000:
    - Camera frame from t=1000
    - IMU reading from t=990 (closest before)
    - Robot pose from t=500 (latched, still valid)
```

### Key Insights

1. **IMU Period Configuration**: Must be >= actual IMU period to avoid ABORTs
2. **Pose Latching**: Handles slow/sparse updates gracefully
3. **Timing Alignment**: All data selected relative to camera timestamp

---

## Workflow 2: Sliding Window Analysis

**Scenario:** Moving average over sensor data with historical context

**Components:**
- `driver::Batch` - Main sensor (sliding window)
- `follower::CountBefore` - Fixed number of prior readings
- `follower::Before` - All auxiliary data in window

### Setup

```cpp
// Main sensor: capture 10 samples, slide by 1
driver::Batch<Dispatch<int, double>, NoLock> sensor{10};

// Temperature: always want last 5 readings before window
follower::CountBefore<Dispatch<int, double>, NoLock> temp{
    5,   // count: exactly 5 elements
    0    // delay
};

// Events: all events before window start
follower::Before<Dispatch<int, Event>, NoLock> events{0};
```

### Execution Trace

```
DATA STATE at t=100:
    Sensor queue: [0,5,10,15,20,25,30,35,40,45,50,55,60,65,70]
    Temp queue:   [-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50,55]
    Events queue: [2,18,33,67]

CAPTURE 1:
    
    DRIVER (Batch{10}):
    ├─ Capture: stamps [0,5,10,15,20,25,30,35,40,45]
    ├─ Range: {0, 45}
    └─ Queue after: [5,10,15,20,25,30,35,40,45,50,55,60,65,70]
        (only stamp 0 removed)

    FOLLOWER 1 (CountBefore{5}):
    ├─ boundary = 0 (range.lower_stamp - delay)
    ├─ Need 5 elements with stamp < 0
    │   └─ Found: [-20,-15,-10,-5] = only 4 elements!
    │
    └─ State: RETRY (need 1 more element < 0)

    OVERALL: State::RETRY

    → Sync fails because temp doesn't have enough historical data

LATER - More temperature data arrives:
    Temp queue: [-25,-20,-15,-10,-5,0,5,10,...,55]

CAPTURE 2:
    
    DRIVER:
    ├─ Queue: [5,10,15,20,25,30,35,40,45,50,55,60,65,70]
    ├─ Capture: stamps [5,10,15,20,25,30,35,40,45,50]
    ├─ Range: {5, 50}
    └─ After: [10,15,20,25,30,35,40,45,50,55,60,65,70]

    FOLLOWER 1 (CountBefore{5}):
    ├─ boundary = 5
    ├─ Elements < 5: [-25,-20,-15,-10,-5,0] = 6 elements
    ├─ Select last 5: [-20,-15,-10,-5,0]
    └─ State: PRIMED

    FOLLOWER 2 (Before):
    ├─ boundary = 5
    ├─ Have element >= 5? Yes (stamp 5)
    ├─ Elements < 5: stamps 2 (events)
    └─ State: PRIMED (captures [Event{2}])

    OVERALL: State::PRIMED

    OUTPUT:
    ├─ Sensor: [5,10,15,20,25,30,35,40,45,50] (10 samples)
    ├─ Temp:   [-20,-15,-10,-5,0] (5 prior readings)
    └─ Events: [Event{2}] (1 event before window)

CAPTURE 3 (sliding window effect):
    
    DRIVER:
    ├─ Queue: [10,15,20,25,30,35,40,45,50,55,60,65,70]
    ├─ Capture: [10,15,20,25,30,35,40,45,50,55]
    ├─ Range: {10, 55}
    │
    │   Note: stamps [10..50] captured AGAIN (overlap!)
    │
    └─ After: [15,20,25,30,35,40,45,50,55,60,65,70]

    OUTPUT:
    ├─ Sensor: [10,15,20,25,30,35,40,45,50,55]
    ├─ Temp:   [-15,-10,-5,0,5] (shifted by 1)
    └─ Events: [Event{2}] (same event, still < 10)
```

---

## Workflow 3: Rate-Limited Processing with State

**Scenario:** High-frequency data with rate limiting and configuration state

**Components:**
- `driver::Throttled` - High-frequency sensor (downsampled)
- `follower::Latched` - Configuration parameters
- `follower::AnyBefore` - Optional diagnostics

### Setup

```cpp
// Sensor at 1000Hz, process at 100Hz (throttle period = 10ms)
driver::Throttled<Dispatch<int, SensorData>, NoLock> sensor{10};

// Config updates infrequently
follower::Latched<Dispatch<int, Config>, NoLock> config{100};

// Optional diagnostic events
follower::AnyBefore<Dispatch<int, Diagnostic>, NoLock> diag{5};
```

### Execution Trace

```
DATA INJECTION (1000Hz):
    for (int t = 0; t < 100; ++t) {
        sensor.inject(t, data);  // t = 0,1,2,3,4,...,99
    }
    
    Sensor queue: [0,1,2,3,4,5,...,99] (100 elements)
    Config queue: [0:"v1"]
    Diag queue:   [15:"warning", 45:"info", 78:"error"]
    
    Sensor.last_captured_ = MIN

CAPTURE 1:
    
    DRIVER (Throttled{10}):
    ├─ target = MIN + 10 ≈ MIN
    ├─ First element >= MIN: stamp 0
    ├─ Range: {0, 0}
    ├─ last_captured_ = 0
    └─ Remove: stamp 0 only
    
    Queue after: [1,2,3,4,...,99]

    FOLLOWER 1 (Latched):
    ├─ boundary = 0 - 100 = -100
    ├─ Find stamp <= -100: NONE
    ├─ latched_.has_value()? NO (first capture)
    │
    │   But wait - stamp 0 is in queue!
    │   Hmm, boundary -100 < all stamps
    │
    └─ State: ABORT? 
    
    ⚠️ PROBLEM: Latched min_period too large!
       Config stamp 0 is > boundary -100
       But we need it for processing!

SOLUTION: Adjust config min_period:
    follower::Latched<...> config{0};  // min_period = 0

CAPTURE 1 (fixed):
    
    DRIVER: Range = {0, 0}
    
    FOLLOWER 1 (Latched{0}):
    ├─ boundary = 0 - 0 = 0
    ├─ Find stamp <= 0: stamp 0 ✓
    └─ State: PRIMED, latched_ = Config{"v1"}

    FOLLOWER 2 (AnyBefore{5}):
    ├─ boundary = 0 - 5 = -5
    ├─ Elements < -5: NONE
    └─ State: PRIMED (empty, OK!)

    OUTPUT:
    ├─ Sensor: [Dispatch{0}]
    ├─ Config: [Dispatch{0, "v1"}]
    └─ Diag:   [] (empty)

CAPTURE 2:
    
    DRIVER (Throttled):
    ├─ target = 0 + 10 = 10
    ├─ Find first >= 10: stamp 10
    ├─ Range: {10, 10}
    └─ Remove: stamps 1-10

    DROPPED: stamps 1,2,3,4,5,6,7,8,9 (within throttle period)
    
    Queue after: [11,12,...,99]

    FOLLOWER 1 (Latched):
    ├─ boundary = 10
    ├─ No new config in queue
    └─ State: PRIMED (uses latched_ = "v1")

    FOLLOWER 2 (AnyBefore{5}):
    ├─ boundary = 10 - 5 = 5
    ├─ Elements < 5: NONE (queue starts at 15)
    │
    │   Wait - what about stamp 15 "warning"?
    │   It's >= 5, so not captured.
    │
    └─ State: PRIMED (empty)

    OUTPUT:
    ├─ Sensor: [Dispatch{10}]
    ├─ Config: [Dispatch{0, "v1"}] (latched)
    └─ Diag:   [] (warning at 15 not yet captured)

CAPTURE 3:
    
    DRIVER: target = 20, captures stamp 20
    DROPPED: stamps 11-19

    FOLLOWER 2 (AnyBefore):
    ├─ boundary = 20 - 5 = 15
    ├─ Elements < 15: NONE (warning is AT 15)
    │
    │   Note: AnyBefore is exclusive (< boundary)
    │
    └─ State: PRIMED (empty)

    ⚠️ The warning at stamp 15 will be captured when boundary > 15

CAPTURE 4:
    
    DRIVER: target = 30, captures stamp 30
    
    FOLLOWER 2:
    ├─ boundary = 30 - 5 = 25
    ├─ Elements < 25: stamp 15 "warning" ✓
    └─ State: PRIMED

    OUTPUT:
    └─ Diag: [Dispatch{15, "warning"}]

FINAL STATE after all captures:
    Captured sensor stamps: 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
    Dropped sensor stamps:  1-9, 11-19, 21-29, ... (81 total dropped)
    
    Processing rate: 10 captures = 100Hz (as intended)
    Data rate: 100 samples = 1000Hz
    Drop rate: 90%
```

---

## Workflow 4: Exact Timestamp Synchronization

**Scenario:** Multiple sensors with hardware-synchronized timestamps

**Components:**
- `driver::Next` - Primary sensor
- `follower::MatchedStamp` - Secondary sensors (same timestamps)

### Setup

```cpp
// All sensors triggered by same hardware clock
driver::Next<Dispatch<int, SensorA>, NoLock> sensor_a;
follower::MatchedStamp<Dispatch<int, SensorB>, NoLock> sensor_b;
follower::MatchedStamp<Dispatch<int, SensorC>, NoLock> sensor_c;
```

### Execution Trace

```
PERFECT SYNCHRONIZATION:

    Sensor A: [100, 200, 300, 400]
    Sensor B: [100, 200, 300, 400]  // Same timestamps
    Sensor C: [100, 200, 300, 400]

CAPTURE 1:
    DRIVER A: range = {100, 100}
    FOLLOWER B: find stamp == 100 → PRIMED
    FOLLOWER C: find stamp == 100 → PRIMED
    
    OUTPUT:
    ├─ A: Dispatch{100, ...}
    ├─ B: Dispatch{100, ...}
    └─ C: Dispatch{100, ...}

    ✓ All three sensors perfectly aligned at t=100

CAPTURE 2, 3, 4: Similar...


IMPERFECT SYNCHRONIZATION (missing data):

    Sensor A: [100, 200, 300, 400]
    Sensor B: [100, 200, 300, 400]
    Sensor C: [100, 300, 400]  // MISSING stamp 200!

CAPTURE 1: range = {100, 100}
    All PRIMED, output aligned at t=100

CAPTURE 2: range = {200, 200}
    DRIVER A: PRIMED
    FOLLOWER B: stamp 200 found → PRIMED
    FOLLOWER C: 
    ├─ Find stamp == 200
    ├─ Queue: [300, 400]
    ├─ oldest(300) > 200
    │   └─ Proves 200 will never arrive
    │
    └─ State: ABORT!

    OVERALL: State::ABORT
    
    ⚠️ Frame at t=200 dropped due to missing sensor C data!

CAPTURE 3: range = {300, 300}
    All PRIMED, output aligned at t=300

RESULT:
    - t=100: Synchronized ✓
    - t=200: DROPPED (sensor C missing)
    - t=300: Synchronized ✓
    - t=400: Synchronized ✓
```

### Handling Missing Data

```cpp
// Option 1: Use AnyAtOrBefore for fault tolerance
follower::AnyAtOrBefore<Dispatch<int, SensorC>, NoLock> sensor_c{0};
// Will return empty if stamp missing, but won't ABORT

// Option 2: Use ClosestBefore with tight tolerance
follower::ClosestBefore<Dispatch<int, SensorC>, NoLock> sensor_c{
    1,  // period = 1 (expect exact match or ±1)
    0   // delay = 0
};
// Will find closest match, may return slightly off timestamp
```

---

## Workflow 5: Batch Processing with Optional Streams

**Scenario:** Process data in chunks with optional auxiliary information

**Components:**
- `driver::Chunk` - Main data batches
- `follower::Before` - Required context
- `follower::AnyBefore` - Optional metadata

### Setup

```cpp
// Process in batches of 100
driver::Chunk<Dispatch<int, DataPacket>, NoLock> data{100};

// Required: header info before each batch
follower::Before<Dispatch<int, Header>, NoLock> headers{0};

// Optional: annotations (may not exist)
follower::AnyBefore<Dispatch<int, Annotation>, NoLock> annotations{0};
```

### Execution Trace

```
DATA STATE:
    Data:        [0..999] (1000 packets)
    Headers:     [0:"H1", 100:"H2", 200:"H3", ...]
    Annotations: [50:"note1", 350:"note2"]  // Sparse!

CAPTURE 1:
    DRIVER (Chunk{100}):
    ├─ Capture: stamps [0..99]
    ├─ Range: {0, 99}
    └─ Remove all captured

    FOLLOWER 1 (Before):
    ├─ boundary = 0
    ├─ Elements < 0: NONE
    ├─ Have element >= 0? YES (stamp 0)
    └─ State: PRIMED (empty capture, but valid)

    FOLLOWER 2 (AnyBefore):
    ├─ boundary = 0
    ├─ Elements < 0: NONE
    └─ State: PRIMED (always, empty OK)

    OUTPUT:
    ├─ Data:        [packets 0-99]
    ├─ Headers:     [] (none before 0)
    └─ Annotations: [] (none before 0)

CAPTURE 2:
    DRIVER:
    ├─ Range: {100, 199}
    └─ Capture packets 100-199

    FOLLOWER 1 (Before):
    ├─ boundary = 100
    ├─ Elements < 100: [Header{0, "H1"}]
    └─ State: PRIMED

    FOLLOWER 2 (AnyBefore):
    ├─ boundary = 100
    ├─ Elements < 100: [Annotation{50, "note1"}]
    └─ State: PRIMED

    OUTPUT:
    ├─ Data:        [packets 100-199]
    ├─ Headers:     [Header{0, "H1"}]
    └─ Annotations: [Annotation{50, "note1"}]

CAPTURE 3:
    DRIVER: Range = {200, 299}

    FOLLOWER 1:
    ├─ boundary = 200
    ├─ Elements < 200: [Header{100, "H2"}]
    └─ State: PRIMED

    FOLLOWER 2:
    ├─ boundary = 200
    ├─ Elements < 200: NONE (note1 already captured)
    └─ State: PRIMED (empty)

    OUTPUT:
    ├─ Data:        [packets 200-299]
    ├─ Headers:     [Header{100, "H2"}]
    └─ Annotations: []  // No annotations in this range

CAPTURE 4:
    DRIVER: Range = {300, 399}

    OUTPUT:
    ├─ Data:        [packets 300-399]
    ├─ Headers:     [Header{200, "H3"}]
    └─ Annotations: [Annotation{350, "note2"}]
    
    ⚠️ Wait - annotation 350 is INSIDE the batch range [300, 399]!
       But boundary = 300, so only elements < 300 captured.
       Annotation 350 will be captured in NEXT batch.

CAPTURE 5:
    DRIVER: Range = {400, 499}

    OUTPUT:
    └─ Annotations: [Annotation{350, "note2"}]  // From previous range
```

---

## Workflow 6: Interpolation with Ranged Data

**Scenario:** Get data points before and after for interpolation

**Components:**
- `driver::Next` - Query timestamps
- `follower::Ranged` - Data for interpolation

### Setup

```cpp
// Query stream (where we need interpolated values)
driver::Next<Dispatch<int, Query>, NoLock> queries;

// Data stream for interpolation (need before + after)
follower::Ranged<Dispatch<int, double>, NoLock> data{0};
```

### Execution Trace

```
DATA STATE:
    Queries: [50, 150, 250]  // Request values at these times
    Data:    [0, 100, 200, 300]  // Known values at these times

CAPTURE 1:
    DRIVER: range = {50, 50}

    FOLLOWER (Ranged):
    ├─ Need: 1 before 50, elements in [50,50], 1 after 50
    │
    ├─ Before 50: stamp 0 ✓
    ├─ In [50,50]: NONE (no exact match)
    ├─ After 50: stamp 100 ✓
    │
    └─ State: PRIMED

    OUTPUT:
    ├─ Query: Dispatch{50, query}
    └─ Data:  [Dispatch{0, v0}, Dispatch{100, v1}]

    Application can now interpolate:
    value_at_50 = v0 + (v1 - v0) * (50 - 0) / (100 - 0)
                = v0 + 0.5 * (v1 - v0)

CAPTURE 2:
    DRIVER: range = {150, 150}

    FOLLOWER:
    ├─ Before 150: stamp 100 ✓
    ├─ After 150: stamp 200 ✓
    └─ State: PRIMED

    OUTPUT:
    └─ Data: [Dispatch{100}, Dispatch{200}]

    Interpolation: value_at_150 = v100 + 0.5 * (v200 - v100)

EDGE CASE - Query before all data:
    Queries: [50]
    Data:    [100, 200, 300]  // Nothing before 50!

    DRIVER: range = {50, 50}
    
    FOLLOWER:
    ├─ Before 50: NONE!
    └─ State: ABORT (cannot interpolate without lower bound)

EDGE CASE - Query after all data:
    Queries: [350]
    Data:    [100, 200, 300]  // Nothing after 350!

    DRIVER: range = {350, 350}
    
    FOLLOWER:
    ├─ Before 350: stamp 300 ✓
    ├─ After 350: NONE (newest is 300)
    └─ State: RETRY (waiting for data after 350)
```

---

## Summary: Workflow Selection Guide

| Scenario | Driver | Primary Follower | Secondary Followers |
|----------|--------|------------------|---------------------|
| Frame-by-frame | `Next` | `ClosestBefore` | `Latched`, `AnyBefore` |
| Sliding window | `Batch` | `Ranged`, `Before` | `CountBefore` |
| Fixed batches | `Chunk` | `Before` | `AnyBefore` |
| Rate limiting | `Throttled` | `Latched` | `AnyBefore` |
| Exact sync | `Next` | `MatchedStamp` | - |
| Interpolation | `Next` | `Ranged` | - |
