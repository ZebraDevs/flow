# Delay Configuration Guide

This document explains how the `delay` parameter works in Flow followers, its configuration options, and practical use cases.

---

## Table of Contents

1. [What is Delay?](#what-is-delay)
2. [Delay in Each Follower Type](#delay-in-each-follower-type)
3. [Boundary Calculation](#boundary-calculation)
4. [Use Cases](#use-cases)
5. [Configuration Examples](#configuration-examples)
6. [Common Pitfalls](#common-pitfalls)

---

## What is Delay?

The `delay` parameter is an **offset** that shifts the capture boundary relative to the driver's timestamp range. It allows followers to look further back (or forward) in time when selecting which data to capture.

```
WITHOUT DELAY (delay = 0):
    
    Driver timestamp:  |------ range -------|
                       lower              upper
                                            ↓
    Follower boundary: ─────────────────────● (at upper_stamp)
    Captures:          ████████████████████
                       (everything before boundary)

WITH DELAY (delay = 5):
    
    Driver timestamp:  |------ range -------|
                       lower              upper
                            ↓
    Follower boundary: ─────● (at upper_stamp - 5)
    Captures:          █████
                       (everything before shifted boundary)
```

### Key Concept

```
boundary = driver_timestamp - delay

A positive delay shifts the boundary EARLIER in time
A zero delay means the boundary equals the driver timestamp
```

---

## Delay in Each Follower Type

All followers (except `MatchedStamp` and `Latched`) accept a `delay` parameter:

### Constructor Signatures

```cpp
// Before: captures all elements before boundary
follower::Before<Dispatch, Lock> before{delay};

// ClosestBefore: captures closest element before boundary within period
follower::ClosestBefore<Dispatch, Lock> closest{period, delay};

// CountBefore: captures N elements before boundary
follower::CountBefore<Dispatch, Lock> count{n, delay};

// Ranged: captures elements in range with bounds
follower::Ranged<Dispatch, Lock> ranged{delay};

// AnyBefore: optionally captures elements before boundary
follower::AnyBefore<Dispatch, Lock> any_before{delay};

// AnyAtOrBefore: optionally captures elements at or before boundary
follower::AnyAtOrBefore<Dispatch, Lock> any_at_before{delay};
```

### Followers WITHOUT Delay Parameter

```cpp
// MatchedStamp: requires exact timestamp match
follower::MatchedStamp<Dispatch, Lock> matched{};  // No delay

// Latched: uses min_period instead
follower::Latched<Dispatch, Lock> latched{min_period};  // Different concept
```

---

## Boundary Calculation

Different followers calculate their boundary differently:

### Upper Stamp Based (Before, CountBefore, AnyBefore, AnyAtOrBefore)

```cpp
// From include/flow/impl/follower/before.hpp
const stamp_type boundary = range.upper_stamp - delay_;
```

```
Driver range: {lower=10, upper=20}
delay = 5

boundary = 20 - 5 = 15

Captures elements with stamp < 15
```

### Lower Stamp Based (ClosestBefore)

```cpp
// From include/flow/impl/follower/closest_before.hpp
const stamp_type boundary = range.lower_stamp - delay_;
```

```
Driver range: {lower=10, upper=20}
delay = 5

boundary = 10 - 5 = 5

Searches for closest element before stamp 5
```

### Both Stamps (Ranged)

```cpp
// From include/flow/impl/follower/ranged.hpp
offset_lower_stamp = range.lower_stamp - delay_;
offset_upper_stamp = range.upper_stamp - delay_;
```

```
Driver range: {lower=10, upper=20}
delay = 5

Shifted range: {lower=5, upper=15}

Captures elements in [5, 15] plus boundary elements
```

---

## Use Cases

### Use Case 1: Sensor Latency Compensation

**Scenario:** Your IMU publishes data with timestamps 10ms behind the actual event time due to processing latency.

```cpp
// Camera frames (driver) at actual time
driver::Next<Dispatch<int, Frame>, NoLock> camera;

// IMU data arrives 10ms late
// Without delay: IMU stamp 90 would be "closest to" camera stamp 100
// With delay=10: looks for IMU data at stamp 90, which is correct!

follower::ClosestBefore<Dispatch<int, ImuData>, NoLock> imu{
    15,   // period: expect data every ~15ms
    10    // delay: compensate for 10ms latency
};
```

```
TIMELINE:
    Real event:     t=100 (camera captures frame)
    Camera stamp:   t=100
    IMU event:      t=100 (same real-world time)
    IMU stamp:      t=90  (delayed by 10ms in timestamp)

WITHOUT DELAY (delay=0):
    boundary = 100 - 0 = 100
    Looks for IMU near stamp 100
    May miss the correct IMU reading!

WITH DELAY (delay=10):
    boundary = 100 - 10 = 90
    Looks for IMU near stamp 90
    Correctly finds the synchronized IMU reading!
```

### Use Case 2: Look-Ahead Buffer

**Scenario:** You need historical context before processing the current frame.

```cpp
// Process frames but need 50ms of prior data for smoothing
driver::Next<Dispatch<int, Frame>, NoLock> frames;

follower::Before<Dispatch<int, SensorData>, NoLock> history{
    50    // delay: capture data up to 50ms before frame
};
```

```
Frame at t=100:
    boundary = 100 - 50 = 50
    Captures all sensor data with stamp < 50
    
    Use case: Moving average, Kalman filter initialization
```

### Use Case 3: Time-Shifted Data Streams

**Scenario:** Two sensors have known constant offset in their timestamp domains.

```cpp
// Sensor A timestamps in system clock
driver::Next<Dispatch<int64_t, DataA>, NoLock> sensor_a;

// Sensor B timestamps offset by +100ms from system clock
// (e.g., different clock source)
follower::ClosestBefore<Dispatch<int64_t, DataB>, NoLock> sensor_b{
    20,    // period
    -100   // NEGATIVE delay: sensor B is 100ms AHEAD
};
```

```
Sensor A stamp: 1000
Sensor B stamp: 1100 (same real-world time)

boundary = 1000 - (-100) = 1100

Correctly aligns with Sensor B's timestamp domain!
```

### Use Case 4: Capturing Prior State

**Scenario:** Robot needs the configuration that was active BEFORE the current command.

```cpp
driver::Next<Dispatch<int, Command>, NoLock> commands;

// Get config that was valid at least 1 second before command
follower::Latched<Dispatch<int, Config>, NoLock> config{1000};  // min_period
```

Note: `Latched` uses `min_period` instead of `delay`, but achieves similar effect:
```cpp
// From Latched implementation
boundary = range.lower_stamp - min_period_;
```

### Use Case 5: Interpolation Data Selection

**Scenario:** Need data points surrounding a target time for interpolation.

```cpp
driver::Next<Dispatch<int, Query>, NoLock> queries;

// Get surrounding points for interpolation
// delay=0 means exact alignment with query timestamps
follower::Ranged<Dispatch<int, Sample>, NoLock> samples{0};
```

```
Query at t=150:
    delay = 0
    range = {150, 150}
    
    Ranged captures:
    - One element before 150 (e.g., stamp 100)
    - Elements in [150, 150] (if any)
    - One element after 150 (e.g., stamp 200)
    
    Result: [100, 200] for linear interpolation
```

---

## Configuration Examples

### Example 1: Basic Delay Setup

```cpp
#include <flow/flow.hpp>

using namespace flow;
using Dispatch = Dispatch<int, double>;

int main() {
    // Driver: process data one at a time
    driver::Next<Dispatch, NoLock> driver;
    
    // Follower with 10-unit delay
    follower::Before<Dispatch, NoLock> follower{10};
    
    // Inject data
    driver.inject(100, 1.0);
    follower.inject(85, 0.85);   // Will be captured (< 100-10=90)
    follower.inject(92, 0.92);   // Will NOT be captured (>= 90)
    follower.inject(95, 0.95);   // Will NOT be captured (>= 90)
    
    Synchronizer sync{driver, follower};
    
    std::vector<Dispatch> driver_data, follower_data;
    State state = sync.capture(
        std::back_inserter(driver_data),
        std::back_inserter(follower_data)
    );
    
    // driver_data: [{100, 1.0}]
    // follower_data: [{85, 0.85}]
    
    return 0;
}
```

### Example 2: ClosestBefore with Period and Delay

```cpp
// IMU at 100Hz (10ms period), with 5ms timestamp delay
follower::ClosestBefore<Dispatch<int, ImuReading>, NoLock> imu{
    10,   // period: maximum expected gap between readings
    5     // delay: timestamp offset compensation
};

// Camera frame at t=1000
// boundary = 1000 - 5 = 995
// window = [995 - 10, 995) = [985, 995)
// Captures closest IMU reading in [985, 995)
```

### Example 3: CountBefore with Delay

```cpp
// Need exactly 5 historical readings, with 20-unit lookback
follower::CountBefore<Dispatch<int, Sample>, NoLock> history{
    5,    // count: exactly 5 elements
    20    // delay: look 20 units back
};

// Driver at t=100
// boundary = 100 - 20 = 80
// Captures 5 most recent elements with stamp < 80
```

### Example 4: Zero Delay (Common Case)

```cpp
// When timestamps are already aligned, use delay=0
follower::Before<Dispatch<int, Data>, NoLock> aligned{0};

// boundary = driver_upper_stamp - 0 = driver_upper_stamp
// Captures everything strictly before driver timestamp
```

---

## Common Pitfalls

### Pitfall 1: Delay Too Large

```cpp
follower::Before<Dispatch<int, Data>, NoLock> follower{1000};  // Large delay
```

```
Driver at t=100:
    boundary = 100 - 1000 = -900
    
    Follower queue: [50, 75, 90, 110]
    
    Elements < -900: NONE
    
    ⚠️ Always captures empty! Delay is too large for the data.
```

**Fix:** Match delay to actual timing relationship between streams.

### Pitfall 2: Negative Delay Confusion

```cpp
// Negative delay shifts boundary FORWARD in time
follower::Before<Dispatch<int, Data>, NoLock> follower{-50};
```

```
Driver at t=100:
    boundary = 100 - (-50) = 150
    
    Follower queue: [50, 75, 90, 110, 130]
    
    Elements < 150: [50, 75, 90, 110, 130]
    
    ⚠️ Captures MORE data than expected!
    ⚠️ May capture data "from the future" relative to driver!
```

**Caution:** Negative delay is valid but can lead to unexpected behavior.

### Pitfall 3: Period vs Delay Confusion (ClosestBefore)

```cpp
follower::ClosestBefore<Dispatch<int, Data>, NoLock> follower{
    5,    // period (window size)
    10    // delay (boundary offset)
};
```

```
These are DIFFERENT concepts:
- period: Size of the search window
- delay: Offset of the boundary

Driver at t=100:
    boundary = 100 - 10 = 90
    window = [90 - 5, 90) = [85, 90)
    
    Looks for closest element in [85, 90)
```

### Pitfall 4: Inconsistent Delay Across Followers

```cpp
// PROBLEM: Different delays cause misalignment
follower::ClosestBefore<...> imu{10, 5};      // delay=5
follower::Before<...> events{20};              // delay=20

// At driver stamp 100:
// IMU boundary:    100 - 5 = 95
// Events boundary: 100 - 20 = 80
//
// These capture data from different time periods!
```

**Fix:** Use consistent delays unless streams have different timing relationships.

### Pitfall 5: Forgetting Delay in Latched

```cpp
// Latched uses min_period, NOT delay
follower::Latched<Dispatch<int, Config>, NoLock> config{100};

// This is NOT the same as delay!
// min_period affects how far back to look for valid config
// boundary = range.lower_stamp - min_period
```

---

## Summary: Delay Parameter Reference

| Follower | Delay Used In | Effect |
|----------|--------------|--------|
| `Before` | `upper_stamp - delay` | Captures elements before shifted boundary |
| `ClosestBefore` | `lower_stamp - delay` | Finds closest element before shifted boundary |
| `CountBefore` | `upper_stamp - delay` | Counts elements before shifted boundary |
| `Ranged` | Both stamps shifted | Entire range shifted by delay |
| `AnyBefore` | `upper_stamp - delay` | Optional capture before shifted boundary |
| `AnyAtOrBefore` | `upper_stamp - delay` | Optional capture at/before shifted boundary |
| `MatchedStamp` | N/A | No delay (exact match required) |
| `Latched` | Uses `min_period` instead | Different concept for staleness |

### Quick Decision Guide

| Situation | Recommended Delay |
|-----------|-------------------|
| Timestamps perfectly aligned | `delay = 0` |
| Follower timestamps late by X | `delay = X` |
| Follower timestamps early by X | `delay = -X` |
| Need historical context | `delay = lookback_time` |
| Unknown timing relationship | Start with `delay = 0`, tune empirically |
