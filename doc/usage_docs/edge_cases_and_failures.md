# Edge Cases and Failure Scenarios

This document covers common failure scenarios, edge cases, and their solutions when using Flow.

---

## Table of Contents

1. [Understanding Capture States](#understanding-capture-states)
2. [ABORT Scenarios](#abort-scenarios)
3. [RETRY Scenarios](#retry-scenarios)
4. [Deadlock and Starvation](#deadlock-and-starvation)
5. [Timestamp Violations](#timestamp-violations)
6. [Configuration Errors](#configuration-errors)
7. [Recovery Strategies](#recovery-strategies)

---

## Understanding Capture States

Flow uses three states to communicate synchronization status:

```cpp
enum class State {
    PRIMED,  // All captors ready, data captured
    RETRY,   // Need more data, try again later
    ABORT    // Data gap detected, skip this frame
};
```

### State Propagation Rules

```
             ┌─────────────────────────────────────────┐
             │       Synchronizer State Logic          │
             ├─────────────────────────────────────────┤
             │                                         │
             │  if (driver == ABORT)  → return ABORT   │
             │  if (any follower == ABORT) → ABORT     │
             │  if (driver == RETRY)  → return RETRY   │
             │  if (any follower == RETRY) → RETRY     │
             │  otherwise             → return PRIMED  │
             │                                         │
             │  Priority: ABORT > RETRY > PRIMED       │
             │                                         │
             └─────────────────────────────────────────┘
```

---

## ABORT Scenarios

ABORT indicates irrecoverable data loss - the frame must be skipped.

### Scenario 1: MatchedStamp Missing Data

```cpp
driver::Next<Dispatch<int, A>, NoLock> driver;
follower::MatchedStamp<Dispatch<int, B>, NoLock> follower;
```

```
DATA STATE:
    Driver:   [100, 200, 300]
    Follower: [100, 300]  // Missing stamp 200!

CAPTURE at t=100:
    Driver range: {100, 100}
    Follower: find(100) → PRIMED ✓

CAPTURE at t=200:
    Driver range: {200, 200}
    Follower:
    ├─ find(200)
    ├─ Queue: [300] (100 already consumed)
    ├─ oldest(300) > 200
    │   └─ If oldest > target, data will NEVER arrive
    │
    └─ State: ABORT

    ⚠️ ABORT: stamp 200 cannot be matched, frame dropped

CAPTURE at t=300:
    Driver range: {300, 300}
    Follower: find(300) → PRIMED ✓

RESULT:
    - t=100: Success
    - t=200: DROPPED
    - t=300: Success
```

### Scenario 2: ClosestBefore No Data in Window

```cpp
follower::ClosestBefore<Dispatch<int, Data>, NoLock> follower{
    10,  // period
    0    // delay
};
```

```
DATA STATE:
    Driver:   [100]
    Follower: [80, 120]  // Gap around 100!

CAPTURE at t=100:
    Driver range: {100, 100}
    Follower:
    ├─ boundary = 100 - 0 = 100
    ├─ window = [100 - 10, 100) = [90, 100)
    │
    ├─ Elements in [90, 100): NONE
    │   80 < 90, 120 >= 100
    │
    ├─ Have element >= 100? YES (120)
    │   → Proves no data [90,100) will arrive
    │
    └─ State: ABORT

FIX: Increase period to cover gap
    follower::ClosestBefore<...> follower{30, 0};  // Window [70, 100)
    Now stamp 80 is captured!
```

### Scenario 3: Before Without Future Proof

```cpp
follower::Before<Dispatch<int, Data>, NoLock> follower{0};
```

```
DATA STATE:
    Driver:   [100, 200]
    Follower: [50]  // Only one element!

CAPTURE 1 at t=100:
    Driver range: {100, 100}
    Follower:
    ├─ boundary = 100
    ├─ Elements < 100: [50] ✓
    ├─ Have element >= 100? NO
    │   └─ Cannot prove no more data < 100 will arrive
    │
    └─ State: RETRY (waiting for proof)

Follower gets stamp 150:
    Follower: [50, 150]

CAPTURE 1 (retry):
    Follower:
    ├─ Elements < 100: [50] ✓
    ├─ Have element >= 100? YES (150)
    │   └─ Proves 50 is all data < 100
    │
    └─ State: PRIMED

    ⚠️ Note: stamp 150 is the "proof" element, not captured
```

### Scenario 4: CountBefore Insufficient Historical Data

```cpp
follower::CountBefore<Dispatch<int, Data>, NoLock> follower{5, 0};
```

```
DATA STATE:
    Driver:   [100]
    Follower: [90, 95, 98]  // Only 3 elements < 100!

CAPTURE:
    Driver range: {100, 100}
    Follower:
    ├─ boundary = 100
    ├─ Elements < 100: [90, 95, 98] (count = 3)
    ├─ Need: 5 elements
    │
    ├─ Have element >= 100? NO
    │   → Still might get more < 100
    │
    └─ State: RETRY

Follower gets stamp 105:
    Follower: [90, 95, 98, 105]

CAPTURE (retry):
    Follower:
    ├─ Elements < 100: [90, 95, 98] (count = 3)
    ├─ Have element >= 100? YES (105)
    │   → Proves only 3 elements exist
    │
    └─ State: ABORT (will NEVER have 5 elements)

    ⚠️ ABORT: Cannot satisfy count requirement
```

---

## RETRY Scenarios

RETRY means more data is needed - synchronization will succeed eventually.

### Scenario 1: Driver Queue Empty

```cpp
driver::Next<Dispatch<int, Data>, NoLock> driver;
```

```
CAPTURE when empty:
    Driver:
    ├─ queue_.empty() == true
    └─ State: RETRY

    Synchronizer returns RETRY, waits for data
```

### Scenario 2: Ranged Waiting for Upper Bound

```cpp
follower::Ranged<Dispatch<int, Data>, NoLock> follower{0};
```

```
DATA STATE:
    Driver:   [100]
    Follower: [80, 90, 100]  // No element AFTER 100!

CAPTURE:
    Driver range: {100, 100}
    Follower:
    ├─ Need: 1 before, elements in [100,100], 1 after
    │
    ├─ Before 100: stamps 80, 90 ✓
    ├─ In [100, 100]: stamp 100 ✓
    ├─ After 100: NONE (newest is 100)
    │   └─ Cannot determine if more data coming
    │
    └─ State: RETRY

Follower gets stamp 110:
    Follower: [80, 90, 100, 110]

CAPTURE (retry):
    Follower:
    ├─ After 100: stamp 110 ✓
    └─ State: PRIMED

    Captured: [90, 100, 110]
    (80 may be removed, 90 kept as lower bound)
```

### Scenario 3: Batch Insufficient Elements

```cpp
driver::Batch<Dispatch<int, Data>, NoLock> driver{10};  // Need 10
```

```
CAPTURE:
    Driver queue: [0, 10, 20, 30]  // Only 4 elements!
    
    Driver:
    ├─ queue_.size() = 4 < min_period (10)
    └─ State: RETRY

    Waits until 10+ elements available
```

### Scenario 4: Latched No Initial Value

```cpp
follower::Latched<Dispatch<int, Config>, NoLock> follower{100};
```

```
DATA STATE (startup):
    Driver:   [100]
    Follower: []  // No config yet!

CAPTURE:
    Driver range: {100, 100}
    Follower:
    ├─ boundary = 100 - 100 = 0
    ├─ Find stamp <= 0: NONE (queue empty)
    ├─ latched_.has_value()? NO (first capture)
    │
    │   Two possibilities:
    │   1. Queue empty: RETRY
    │   2. Have newer data but nothing <= boundary: ABORT
    │
    └─ State: RETRY (queue empty, might get data)

Follower gets stamp 50:
    Follower: [50]

CAPTURE (retry):
    Driver range: {100, 100}
    Follower:
    ├─ boundary = 0
    ├─ Find stamp <= 0: NONE
    ├─ But have stamp 50 > 0
    │   → Proves nothing <= 0 exists
    │
    └─ State: ABORT? 

    ⚠️ This depends on implementation details!
    
    FIX: Ensure config arrives before driver data
         OR set min_period appropriately
```

---

## Deadlock and Starvation

### Deadlock: Circular Dependency

```
SCENARIO:
    System A sends data to System B
    System B sends data to System A
    Both waiting for each other!

    Synchronizer 1:
    ├─ Driver: System A output
    └─ Follower: System B output (RETRY - waiting)

    Synchronizer 2:
    ├─ Driver: System B output  
    └─ Follower: System A output (RETRY - waiting)

    DEADLOCK: Neither can make progress!

SOLUTION:
    1. Use separate threads for each synchronizer
    2. Add timeout mechanism
    3. Restructure to break circular dependency
```

### Starvation: Slow Follower

```cpp
driver::Next<Dispatch<int, Fast>, NoLock> fast_driver;  // 1000Hz
follower::Before<Dispatch<int, Slow>, NoLock> slow_follower{0};  // 1Hz
```

```
SCENARIO:
    Fast driver: stamps [0, 1, 2, 3, ..., 999]
    Slow follower: stamp [0] only

CAPTURE 1 at t=0:
    Driver range: {0, 0}
    Follower:
    ├─ boundary = 0
    ├─ Elements < 0: NONE
    ├─ Have element >= 0? YES (stamp 0)
    └─ State: PRIMED (empty capture OK)

CAPTURE 2 at t=1:
    Driver range: {1, 1}
    Follower:
    ├─ boundary = 1
    ├─ Elements < 1: [0]
    ├─ Have element >= 1? NO
    │
    └─ State: RETRY

    Fast driver queue grows: [2, 3, 4, ..., 999, 1000, ...]
    Synchronizer blocked waiting for slow follower!

SOLUTION:
    1. Use Throttled driver to match slow rate
    2. Use AnyBefore (won't block on empty)
    3. Buffer management in application layer
```

### Starvation: Throttled Skipping All Data

```cpp
driver::Throttled<Dispatch<int, Data>, NoLock> driver{1000};  // 1Hz output
```

```
DATA STATE:
    Data arrives at 10Hz: [0, 100, 200, 300, ...]

CAPTURE 1:
    last_captured_ = MIN
    target = MIN + 1000 ≈ MIN
    First element >= MIN: stamp 0
    Captured: [0]
    last_captured_ = 0

CAPTURE 2:
    target = 0 + 1000 = 1000
    Data queue: [100, 200, 300, 400, 500, 600, 700, 800, 900]
    
    Driver:
    ├─ Find first >= 1000: NONE (all < 1000)
    └─ State: RETRY

    ... time passes, more data arrives ...

    Data queue: [100, 200, ..., 900, 1000, 1100]

CAPTURE 2 (retry):
    Find first >= 1000: stamp 1000
    Captured: [1000]
    DROPPED: stamps 100-900 (skipped by throttle)

RESULT:
    Captured: 0, 1000, 2000, ...
    Dropped: everything in between

    ⚠️ If data rate < throttle rate:
       Most data captured, some RETRY delays
    
    ⚠️ If data rate > throttle rate:
       Significant data dropped (by design!)
```

---

## Timestamp Violations

### Non-Monotonic Timestamps

```cpp
// Flow assumes timestamps are monotonically increasing!
captor.inject(100, data1);
captor.inject(200, data2);
captor.inject(150, data3);  // VIOLATION: 150 < 200
```

```
QUEUE STATE:
    After inject(100): [100]
    After inject(200): [100, 200]
    After inject(150): [100, 200, 150]  // Wrong order!

CAPTURE:
    Driver assumes queue is sorted!
    
    oldest_stamp() may return wrong value
    Binary search may fail
    Capture ranges may be incorrect
    
    ⚠️ UNDEFINED BEHAVIOR

SOLUTION:
    1. Sort data before injection
    2. Use application-level timestamping
    3. Add validation layer:
    
    void safe_inject(Stamp stamp, Data data) {
        if (!queue_.empty() && stamp < queue_.back().stamp()) {
            // Handle: discard, reorder, or error
        }
        captor.inject(stamp, data);
    }
```

### Timestamp Overflow

```cpp
using Stamp = int32_t;  // Limited range!

// Near overflow
captor.inject(2147483647, data);  // INT32_MAX
captor.inject(2147483647 + 1, data);  // Overflow to negative!
```

```
QUEUE STATE:
    [2147483647, -2147483648]  // Apparent time reversal!

SOLUTION:
    1. Use int64_t or uint64_t timestamps
    2. Use relative timestamps with epoch reset
    3. Implement wrap-around handling
```

### Very Large Timestamp Gaps

```cpp
driver::Throttled<Dispatch<int64_t, Data>, NoLock> driver{1000};
```

```
DATA STATE:
    Normal:     [0, 1, 2, ..., 999]
    Then gap:   [0, 1, ..., 999, 1000000]  // Jump of ~1 second to ~17 minutes

CAPTURE after gap:
    last_captured_ = 999
    target = 999 + 1000 = 1999
    
    Find first >= 1999: stamp 1000000
    
    ├─ Captures stamp 1000000
    └─ last_captured_ = 1000000

NEXT CAPTURE:
    target = 1000000 + 1000 = 1001000
    
    ⚠️ If data resumes at normal rate:
       stamps [1001, 1002, ...] all < 1001000
       All SKIPPED until stamp >= 1001000!

SOLUTION:
    1. Detect and handle gaps in application
    2. Reset synchronizer on large gaps
    3. Use adaptive throttle period
```

---

## Configuration Errors

### Period Too Small

```cpp
follower::ClosestBefore<Dispatch<int, Data>, NoLock> follower{1, 0};
// Period = 1, but data arrives every 10ms
```

```
DATA STATE:
    Data: [0, 10, 20, 30, ...]  // 10ms intervals

CAPTURE at t=100:
    Driver range: {100, 100}
    Follower:
    ├─ boundary = 100, window = [99, 100)
    ├─ No data in [99, 100)!
    │   (stamp 100 is >= 100, not in window)
    │
    └─ State: ABORT

EVERY capture fails!

FIX: period >= data interval
    follower::ClosestBefore<...> follower{15, 0};  // Window [85, 100)
    Now captures stamp 90 or 100
```

### Delay Misconfiguration

```cpp
follower::Before<Dispatch<int, Data>, NoLock> follower{100};
// Delay = 100, expects data 100ms before driver
```

```
DATA STATE:
    Driver:   [100, 200, 300]
    Follower: [100, 200, 300]  // Same timestamps as driver!

CAPTURE at t=100:
    Driver range: {100, 100}
    Follower:
    ├─ boundary = 100 - 100 = 0
    ├─ Elements < 0: NONE
    │
    └─ No data captured (all follower stamps > 0)

    ⚠️ Delay doesn't match actual data relationship!

FIX: Match delay to actual timing relationship
    follower::Before<...> follower{0};  // Same timestamps
```

### Count Larger Than Available

```cpp
follower::CountBefore<Dispatch<int, Data>, NoLock> follower{100, 0};
// Requires 100 historical elements
```

```
DATA STATE:
    Stream only produces 10 elements total!

RESULT:
    Every capture attempt → ABORT
    System never makes progress

FIX: Set count to realistic value
    follower::CountBefore<...> follower{5, 0};
```

---

## Recovery Strategies

### Strategy 1: Retry with Timeout

```cpp
template<typename Sync>
State capture_with_timeout(Sync& sync, 
                          std::chrono::milliseconds timeout) {
    auto start = std::chrono::steady_clock::now();
    
    while (true) {
        State state = sync.capture(...);
        
        if (state != State::RETRY) {
            return state;
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > timeout) {
            return State::ABORT;  // Give up
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
```

### Strategy 2: Skip ABORTs and Continue

```cpp
while (true) {
    State state = sync.capture(...);
    
    switch (state) {
        case State::PRIMED:
            process_data(...);
            break;
            
        case State::RETRY:
            // Wait for more data
            wait_for_data();
            break;
            
        case State::ABORT:
            // Log and continue
            log_dropped_frame();
            // Data already removed, next capture will try new data
            break;
    }
}
```

### Strategy 3: Fallback to Partial Data

```cpp
// Use AnyBefore/AnyAtOrBefore for optional data
follower::AnyBefore<Dispatch<int, Optional>, NoLock> optional{0};

// These never ABORT - empty result is valid
// Application handles missing data gracefully
```

### Strategy 4: Dynamic Reconfiguration

```cpp
// Monitor ABORT rate
int abort_count = 0;
int total_count = 0;

while (true) {
    State state = sync.capture(...);
    total_count++;
    
    if (state == State::ABORT) {
        abort_count++;
        
        float abort_rate = float(abort_count) / total_count;
        if (abort_rate > 0.5) {  // >50% failures
            // Reconfigure: increase periods, reduce requirements
            reconfigure_captors();
            abort_count = 0;
            total_count = 0;
        }
    }
}
```

---

## Summary: Common Pitfalls

| Issue | Symptom | Solution |
|-------|---------|----------|
| MatchedStamp missing data | ABORT | Use ClosestBefore or AnyBefore |
| Period too small | Frequent ABORT | Increase period to match data rate |
| Count too large | Permanent ABORT | Reduce count or use Before |
| Delay mismatch | Empty captures | Match delay to actual timing |
| Slow follower | RETRY forever | Use Throttled driver or AnyBefore |
| Non-monotonic timestamps | Undefined behavior | Sort before inject |
| Empty queues at start | RETRY | Pre-populate or handle startup |
| Large timestamp gaps | Skipped data | Detect gaps, reset state |
