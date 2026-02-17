# Boundary Calculation Reference

This document clarifies how each follower calculates its capture boundary, based on analysis of the actual implementation code.

---

## Table of Contents

1. [Overview](#overview)
2. [Documentation vs Implementation Discrepancy](#documentation-vs-implementation-discrepancy)
3. [Actual Boundary Calculations](#actual-boundary-calculations)
4. [Why the Difference Matters](#why-the-difference-matters)
5. [Practical Implications](#practical-implications)

---

## Overview

Each follower type calculates a **boundary** that determines which elements to capture. The boundary is calculated from the driver's `CaptureRange`, which contains:

```cpp
struct CaptureRange<StampT> {
    StampT lower_stamp;  // Oldest timestamp in driver's capture
    StampT upper_stamp;  // Newest timestamp in driver's capture
};
```

For `driver::Next`, these are equal (`lower_stamp == upper_stamp`).
For `driver::Batch` or `driver::Chunk`, they differ.

---

## Documentation vs Implementation Discrepancy

⚠️ **Important Finding:** The README.md documentation states that most followers use `lower_stamp`, but the actual implementation uses `upper_stamp` for several followers.

| Follower | README Says | Actual Code |
|----------|-------------|-------------|
| `Before` | `lower_stamp - delay` | **`upper_stamp - delay`** |
| `AnyBefore` | `lower_stamp - delay` | **`upper_stamp - delay`** |
| `AnyAtOrBefore` | `lower_stamp - delay` | **`upper_stamp - delay`** |
| `CountBefore` | `lower_stamp - delay` | **`upper_stamp - delay`** |
| `ClosestBefore` | `lower_stamp - delay` | `lower_stamp - delay` ✓ |
| `Latched` | `lower_stamp - min_period` | `lower_stamp - min_period` ✓ |
| `MatchedStamp` | `lower_stamp` | `lower_stamp` ✓ |
| `Ranged` | Both stamps | Both stamps ✓ |

---

## Actual Boundary Calculations

Based on the implementation in `include/flow/impl/follower/*.hpp`:

### Followers Using `upper_stamp`

#### Before
```cpp
// include/flow/impl/follower/before.hpp, line 55
const stamp_type boundary = range.upper_stamp - delay_;
```
Captures: All elements with `stamp < boundary`

#### AnyBefore
```cpp
// include/flow/impl/follower/any_before.hpp, line 48
const stamp_type boundary = range.upper_stamp - delay_;
```
Captures: All elements with `stamp < boundary` (or empty, always PRIMED)

#### AnyAtOrBefore
```cpp
// include/flow/impl/follower/any_at_or_before.hpp, line 49
const stamp_type boundary = range.upper_stamp - delay_;
```
Captures: All elements with `stamp <= boundary` (or empty, always PRIMED)

#### CountBefore
```cpp
// include/flow/impl/follower/count_before.hpp, line 61
const stamp_type boundary = range.upper_stamp - delay_;
```
Captures: N elements with `stamp < boundary`

### Followers Using `lower_stamp`

#### ClosestBefore
```cpp
// include/flow/impl/follower/closest_before.hpp, line 44
const stamp_type boundary = range.lower_stamp - delay_;
```
Captures: One element in window `[boundary - period, boundary)`

#### Latched
```cpp
// include/flow/impl/follower/latched.hpp
const stamp_type boundary = range.lower_stamp - min_period_;
```
Captures: Most recent element with `stamp <= boundary`

#### MatchedStamp
```cpp
// Uses range.lower_stamp directly for exact match
```
Captures: Element with `stamp == range.lower_stamp`

### Followers Using Both Stamps

#### Ranged
```cpp
// include/flow/impl/follower/ranged.hpp, lines 114, 135
offset_lower_stamp = range.lower_stamp - delay_;
offset_upper_stamp = range.upper_stamp - delay_;
```
Captures: Elements spanning `[lower - delay, upper - delay]` plus boundary elements

---

## Why the Difference Matters

### For `driver::Next` (Single Element)

When the driver captures a single element:
```
range = {lower_stamp=100, upper_stamp=100}
```

Both `lower_stamp` and `upper_stamp` are equal, so **it doesn't matter** which one is used.

### For `driver::Batch` or `driver::Chunk` (Multiple Elements)

When the driver captures a range of elements:
```
range = {lower_stamp=100, upper_stamp=200}
```

Now the choice matters significantly:

```
Timeline:
    50      100              200     250
    |--------|----------------|-------|
             ^                ^
           lower            upper


Using upper_stamp (Before, AnyBefore, CountBefore):
    boundary = 200 - delay
    
    Captures data up to timestamp 200 (END of driver range)
    This INCLUDES data that occurred DURING the driver's capture period!


Using lower_stamp (ClosestBefore, Latched, MatchedStamp):
    boundary = 100 - delay
    
    Captures data up to timestamp 100 (START of driver range)
    This captures data from BEFORE the driver's capture period began.
```

### Visual Example

```
Driver (Batch): captures images at t=[100, 200]

Follower data:  [50, 75, 90, 110, 150, 180, 210, 250]

BEFORE (boundary = upper = 200, delay = 0):
    Captures: [50, 75, 90, 110, 150, 180]
              ├─ Before driver started: 50, 75, 90
              └─ DURING driver capture: 110, 150, 180  ← Included!

CLOSEST_BEFORE (boundary = lower = 100, delay = 0, period = 20):
    Window: [80, 100)
    Captures: [90]
              └─ Only data from BEFORE driver started
```

---

## Practical Implications

### When to Use Each Type

| Use Case | Recommended Follower | Why |
|----------|---------------------|-----|
| "All context up to now" | `Before` | Uses `upper_stamp`, gets everything including during driver capture |
| "State when driver started" | `ClosestBefore`, `Latched` | Uses `lower_stamp`, gets state at beginning |
| "Exact synchronization" | `MatchedStamp` | Uses `lower_stamp` for exact match |
| "Optional data, any available" | `AnyBefore` | Uses `upper_stamp`, never blocks |
| "Historical count" | `CountBefore` | Uses `upper_stamp`, N most recent before end |
| "Interpolation bounds" | `Ranged` | Uses both, spans entire range |

### Common Gotcha with Batch/Chunk Drivers

If you're using `driver::Batch` or `driver::Chunk` and you want follower data from BEFORE the batch started (not during), use:
- `ClosestBefore` - for single element
- `Latched` - for persistent state
- `MatchedStamp` - for exact timestamp

If you're okay with (or want) data from during the batch period, use:
- `Before` - for all elements
- `CountBefore` - for N elements
- `AnyBefore` / `AnyAtOrBefore` - for optional data

### Example: Sensor Fusion with Batch Driver

```cpp
// Driver captures 10 images at a time
driver::Batch<Dispatch<int, Image>, NoLock> camera{10};

// WRONG (if you want IMU state at batch START):
// Before uses upper_stamp, so you get IMU data from during the batch too
follower::Before<Dispatch<int, ImuData>, NoLock> imu{0};

// CORRECT (for IMU state at batch START):
// ClosestBefore uses lower_stamp
follower::ClosestBefore<Dispatch<int, ImuData>, NoLock> imu{10, 0};
```

---

## Summary Table

| Follower | Boundary Formula | Stamp Used | Semantic Meaning |
|----------|-----------------|------------|------------------|
| `Before` | `upper_stamp - delay` | upper | "Everything before the END" |
| `AnyBefore` | `upper_stamp - delay` | upper | "Anything before the END (optional)" |
| `AnyAtOrBefore` | `upper_stamp - delay` | upper | "Anything at/before the END (optional)" |
| `CountBefore` | `upper_stamp - delay` | upper | "N elements before the END" |
| `ClosestBefore` | `lower_stamp - delay` | lower | "Closest to the START" |
| `Latched` | `lower_stamp - min_period` | lower | "State at the START" |
| `MatchedStamp` | `lower_stamp` | lower | "Exact match at START" |
| `Ranged` | Both stamps - delay | both | "Spanning the entire range" |

---

## Note on Documentation

The main `README.md` in the repository states that followers use `lower_stamp`, but the actual implementation for `Before`, `AnyBefore`, `AnyAtOrBefore`, and `CountBefore` uses `upper_stamp`. This document reflects the **actual implementation behavior** as of the current codebase.
