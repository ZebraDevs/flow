# Message Dropping Behavior

This document details when and why messages are dropped in Flow, including intentional throttling, boundary cleanup, and edge cases.

---

## Table of Contents

1. [Categories of Message Dropping](#categories-of-message-dropping)
2. [Driver-Specific Dropping](#driver-specific-dropping)
3. [Follower-Specific Dropping](#follower-specific-dropping)
4. [Boundary and Proof Elements](#boundary-and-proof-elements)
5. [Synchronizer-Level Dropping](#synchronizer-level-dropping)
6. [Monitoring and Debugging](#monitoring-and-debugging)

---

## Categories of Message Dropping

Messages can be dropped for several reasons:

```
┌─────────────────────────────────────────────────────────────────┐
│                   Message Dropping Categories                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. INTENTIONAL DROPPING (by design)                            │
│     ├─ Throttled driver rate limiting                           │
│     ├─ AnyBefore/AnyAtOrBefore non-deterministic selection      │
│     └─ Batch overlap (only oldest removed per capture)          │
│                                                                  │
│  2. BOUNDARY CLEANUP (after successful capture)                 │
│     ├─ Elements older than captured range                       │
│     ├─ "Proof" elements that enabled capture                    │
│     └─ Elements no longer needed for future captures            │
│                                                                  │
│  3. ABORT SCENARIOS (data loss)                                 │
│     ├─ MatchedStamp missing exact match                         │
│     ├─ ClosestBefore gap in window                              │
│     └─ CountBefore insufficient count                           │
│                                                                  │
│  4. QUEUE OVERFLOW (application-level)                          │
│     └─ Custom queue limits in application code                  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Driver-Specific Dropping

### Next Driver: Minimal Dropping

```cpp
driver::Next<Dispatch<int, Data>, NoLock> driver;
```

**Behavior:** Removes exactly ONE element per capture.

```
CAPTURE SEQUENCE:
    Queue: [100, 200, 300, 400, 500]

    Capture 1: Removes 100 → Queue: [200, 300, 400, 500]
    Capture 2: Removes 200 → Queue: [300, 400, 500]
    Capture 3: Removes 300 → Queue: [400, 500]
    
    TOTAL DROPPED: 0
    TOTAL CAPTURED: 3
    
    ✓ Every message is captured
```

### Batch Driver: Overlap Dropping

```cpp
driver::Batch<Dispatch<int, Data>, NoLock> driver{5};  // batch size 5
```

**Behavior:** Captures N elements, removes only the OLDEST.

```
CAPTURE SEQUENCE:
    Queue: [0, 10, 20, 30, 40, 50, 60, 70, 80]

    Capture 1:
    ├─ Captures: [0, 10, 20, 30, 40]
    ├─ Range: {0, 40}
    ├─ Removes: 0 (only oldest)
    └─ Queue after: [10, 20, 30, 40, 50, 60, 70, 80]

    Capture 2:
    ├─ Captures: [10, 20, 30, 40, 50]  ← stamps 10-40 RECAPTURED!
    ├─ Range: {10, 50}
    ├─ Removes: 10
    └─ Queue after: [20, 30, 40, 50, 60, 70, 80]

    OVERLAP PATTERN:
    ├─ Capture 1: [0, 10, 20, 30, 40]
    ├─ Capture 2: [10, 20, 30, 40, 50]
    ├─ Capture 3: [20, 30, 40, 50, 60]
    └─ ...
    
    Each message captured 5 times before final removal!
    
    TOTAL DROPPED: 0 (all captured, some multiple times)
```

### Chunk Driver: Complete Batch Dropping

```cpp
driver::Chunk<Dispatch<int, Data>, NoLock> driver{5};
```

**Behavior:** Captures N elements, removes ALL captured.

```
CAPTURE SEQUENCE:
    Queue: [0, 10, 20, 30, 40, 50, 60, 70, 80]

    Capture 1:
    ├─ Captures: [0, 10, 20, 30, 40]
    ├─ Removes: [0, 10, 20, 30, 40]
    └─ Queue after: [50, 60, 70, 80]

    Capture 2:
    ├─ Queue size: 4 < 5
    └─ State: RETRY (waiting for more data)

    Queue grows: [50, 60, 70, 80, 90]

    Capture 2 (retry):
    ├─ Captures: [50, 60, 70, 80, 90]
    ├─ Removes: [50, 60, 70, 80, 90]
    └─ Queue after: []

    TOTAL DROPPED: 0
    TOTAL CAPTURED: 10 (in 2 batches)
    
    ✓ Every message captured exactly once
```

### Throttled Driver: Intentional Dropping

```cpp
driver::Throttled<Dispatch<int, Data>, NoLock> driver{100};  // period 100
```

**Behavior:** Captures at target rate, skips intermediate data.

```
CAPTURE SEQUENCE:
    Queue: [0, 10, 20, 30, ..., 990]  // 100 elements at 10ms intervals
    
    Initial: last_captured_ = MIN (effectively -∞)

    Capture 1:
    ├─ target = MIN + 100 ≈ MIN
    ├─ First element >= MIN: stamp 0
    ├─ Captures: [Dispatch{0}]
    ├─ Removes: [0]
    └─ last_captured_ = 0

    Capture 2:
    ├─ target = 0 + 100 = 100
    ├─ First element >= 100: stamp 100
    ├─ Captures: [Dispatch{100}]
    ├─ Removes: [10, 20, 30, ..., 100]  ← 10 elements!
    │
    │   ⚠️ DROPPED: stamps 10, 20, 30, 40, 50, 60, 70, 80, 90
    │      These fell within the throttle period
    │
    └─ last_captured_ = 100

    Capture 3:
    ├─ target = 100 + 100 = 200
    ├─ Captures: [Dispatch{200}]
    ├─ Removes: [110, 120, ..., 200]
    │
    │   ⚠️ DROPPED: 9 more elements
    │
    └─ last_captured_ = 200

    FINAL STATISTICS:
    ├─ Captured: 0, 100, 200, 300, ..., 900 (10 elements)
    ├─ Dropped: 10, 20, 30, ..., 90, 110, 120, ... (90 elements)
    │
    ├─ Capture rate: 10%
    └─ Drop rate: 90%
    
    ⚠️ This is BY DESIGN for rate limiting!
```

### Throttle Period vs Data Rate

```
┌─────────────────────────────────────────────────────────────────┐
│                 Throttle Period Selection                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Data Rate: 100 Hz (10ms between samples)                       │
│                                                                  │
│  ┌────────────────┬──────────────┬───────────────┐              │
│  │ Throttle Period│ Output Rate  │ Drop Rate     │              │
│  ├────────────────┼──────────────┼───────────────┤              │
│  │ 10ms           │ 100 Hz       │ 0% (all kept) │              │
│  │ 20ms           │ 50 Hz        │ 50%           │              │
│  │ 50ms           │ 20 Hz        │ 80%           │              │
│  │ 100ms          │ 10 Hz        │ 90%           │              │
│  │ 500ms          │ 2 Hz         │ 98%           │              │
│  └────────────────┴──────────────┴───────────────┘              │
│                                                                  │
│  Rule: Drop Rate = 1 - (Data_Interval / Throttle_Period)        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Follower-Specific Dropping

### Before: Drops All Captured + Proof

```cpp
follower::Before<Dispatch<int, Data>, NoLock> follower{0};
```

```
CAPTURE:
    Queue: [50, 80, 100, 150]
    Driver range: {100, 100}
    Boundary: 100

    Locate:
    ├─ Elements < 100: [50, 80]  ← CAPTURED
    ├─ Proof element >= 100: 100  ← PROOF (not captured)
    └─ State: PRIMED

    After capture:
    ├─ Removes: [50, 80] (captured)
    └─ Queue: [100, 150]

    Stamp 100 retained (might be proof for next capture)
```

### ClosestBefore: Drops Window + Older

```cpp
follower::ClosestBefore<Dispatch<int, Data>, NoLock> follower{20, 0};
```

```
CAPTURE:
    Queue: [50, 70, 85, 95, 100, 110]
    Driver range: {100, 100}
    Boundary: 100, Window: [80, 100)

    Locate:
    ├─ In window [80, 100): [85, 95]
    ├─ Closest to 100: stamp 95  ← CAPTURED
    └─ State: PRIMED

    After capture:
    ├─ Removes up to boundary: [50, 70, 85, 95]
    │
    │   ⚠️ DROPPED: 50, 70, 85 (older than closest)
    │      These were in queue but not captured
    │
    └─ Queue: [100, 110]

    Only stamp 95 was "used" - others dropped!
```

### CountBefore: Drops Excess Historical

```cpp
follower::CountBefore<Dispatch<int, Data>, NoLock> follower{3, 0};
```

```
CAPTURE:
    Queue: [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110]
    Driver range: {100, 100}
    Boundary: 100

    Locate:
    ├─ Elements < 100: [10, 20, 30, 40, 50, 60, 70, 80, 90]
    ├─ Need: 3 elements
    ├─ Select LAST 3: [70, 80, 90]  ← CAPTURED
    └─ State: PRIMED

    After capture:
    ├─ Removes up to boundary: all stamps < 100
    │
    │   ⚠️ DROPPED: 10, 20, 30, 40, 50, 60 (excess historical)
    │      These existed but weren't part of the count
    │
    └─ Queue: [100, 110]
```

### Ranged: Keeps Boundary Elements

```cpp
follower::Ranged<Dispatch<int, Data>, NoLock> follower{0};
```

```
CAPTURE:
    Queue: [70, 80, 90, 100, 110, 120]
    Driver range: {90, 100}
    Boundary: 90

    Locate:
    ├─ Need: 1 before 90, elements in [90,100], 1 after 100
    │
    ├─ Before 90: stamp 80  ← CAPTURED (lower bound)
    ├─ In [90, 100]: [90, 100]  ← CAPTURED
    ├─ After 100: stamp 110  ← CAPTURED (upper bound)
    │
    └─ Captured: [80, 90, 100, 110]

    After capture:
    ├─ Removes only stamps < 80 (lower bound)
    │
    │   ⚠️ DROPPED: 70 (before lower bound)
    │
    └─ Queue: [80, 90, 100, 110, 120]
    
    Note: 80 kept for potential next capture's upper bound!
```

### AnyBefore: Non-Deterministic Capture

```cpp
follower::AnyBefore<Dispatch<int, Data>, NoLock> follower{0};
```

```
CAPTURE:
    Queue: [50, 60, 70, 80, 90, 100]
    Driver range: {100, 100}
    Boundary: 100

    Locate:
    ├─ Elements < 100: [50, 60, 70, 80, 90]
    │
    │   Implementation may capture:
    │   - ALL elements < boundary
    │   - Just ONE element < boundary
    │   - SOME elements < boundary
    │
    │   ⚠️ Behavior varies by implementation!
    │
    └─ State: PRIMED (even if empty)

    Typical after capture:
    ├─ Removes all <= boundary
    │
    │   ⚠️ Some elements may be CAPTURED
    │   ⚠️ Some elements may be DROPPED (not returned)
    │      depending on implementation
    │
    └─ Queue: [100]

    Use AnyBefore when:
    - You only need to know IF data exists before
    - You can handle non-deterministic capture
    - Missing data is acceptable
```

### Latched: Historical Dropping

```cpp
follower::Latched<Dispatch<int, Config>, NoLock> follower{100};
```

```
CAPTURE:
    Queue: [0, 50, 100, 150]
    Driver range: {200, 200}
    Boundary: 200 - 100 = 100

    Locate:
    ├─ Find stamp <= 100: stamps 0, 50, 100
    ├─ Select MOST RECENT: stamp 100  ← CAPTURED
    ├─ Update latched_: Dispatch{100}
    └─ State: PRIMED

    After capture:
    ├─ Removes stamps <= 100: [0, 50, 100]
    │
    │   ⚠️ DROPPED: 0, 50 (superseded by 100)
    │      Older values no longer needed
    │
    └─ Queue: [150]

SUBSEQUENT CAPTURE:
    Queue: [150]  (no new config)
    Driver range: {300, 300}
    Boundary: 200

    Locate:
    ├─ Find stamp <= 200: NONE in queue
    ├─ Use latched_: Dispatch{100}
    └─ State: PRIMED

    OUTPUT: Returns latched value, nothing dropped
```

---

## Boundary and Proof Elements

### What is a "Proof" Element?

A proof element demonstrates that no more data will arrive before a boundary.

```
PROOF CONCEPT:

    Assumption: Timestamps are monotonically increasing
    
    If queue contains element with stamp >= boundary,
    then NO future element can have stamp < boundary
    
    ├─ Current queue: [50, 80, 120, 150]
    ├─ Boundary: 100
    ├─ Proof element: 120 (first >= 100)
    │
    └─ Conclusion: [50, 80] is ALL data < 100
       (Any future inject will have stamp > newest = 150)
```

### Proof Element Retention

```
SCENARIO: Multiple captures sharing proof

    Queue: [80, 100, 120]
    
    Capture 1 (boundary = 90):
    ├─ Captured: [80]
    ├─ Proof: 100
    └─ Queue after: [100, 120]  // 100 retained!

    Capture 2 (boundary = 110):
    ├─ Captured: [100]  // Previously proof, now data
    ├─ Proof: 120
    └─ Queue after: [120]  // 120 retained!

    The proof element from capture N may become
    captured data in capture N+1
```

---

## Synchronizer-Level Dropping

### ABORT Causes Coordinated Dropping

```cpp
Synchronizer<...> sync{driver, follower1, follower2};
```

```
SCENARIO:
    Driver queue:    [100, 200, 300]
    Follower 1:      [100, 200, 300]
    Follower 2:      [100, 300]  // MISSING 200

CAPTURE for t=200:
    Driver:   range = {200, 200}, ready to capture
    Follower 1: ready to capture
    Follower 2: ABORT (missing stamp 200)

    SYNCHRONIZER:
    ├─ State = ABORT (follower 2 failed)
    ├─ All captors release data for range {200, 200}
    │
    │   Driver: removes stamp 200
    │   Follower 1: removes stamp 200
    │   Follower 2: removes stamps up to 200 (if any)
    │
    └─ Data for t=200 DROPPED from all captors

    ⚠️ Even though driver and follower 1 HAD the data,
       it's dropped because follower 2 couldn't sync
```

### Partial vs Complete Drops

```
┌─────────────────────────────────────────────────────────────────┐
│              Drop Behavior by Synchronization State             │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  PRIMED (success):                                               │
│  ├─ Driver data: CAPTURED and removed                           │
│  └─ Follower data: CAPTURED and removed                         │
│                                                                  │
│  ABORT (failure):                                                │
│  ├─ Driver data: DROPPED (removed without capture)              │
│  └─ Follower data: DROPPED (removed without capture)            │
│                                                                  │
│  RETRY (waiting):                                                │
│  ├─ Driver data: RETAINED                                       │
│  └─ Follower data: RETAINED                                     │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Monitoring and Debugging

### Tracking Dropped Messages

```cpp
// Custom captor wrapper with drop tracking
template<typename Captor>
class MonitoredCaptor : public Captor {
public:
    size_t injected_count = 0;
    size_t captured_count = 0;
    size_t dropped_count = 0;
    
    void inject(auto stamp, auto data) {
        injected_count++;
        Captor::inject(stamp, data);
    }
    
    // Override capture methods to track
    // captured_count and dropped_count
    
    void report() {
        std::cout << "Injected: " << injected_count << "\n"
                  << "Captured: " << captured_count << "\n"
                  << "Dropped:  " << dropped_count << "\n"
                  << "Drop Rate: " 
                  << (100.0 * dropped_count / injected_count) 
                  << "%\n";
    }
};
```

### Debug Output Example

```cpp
#include <flow/synchronizer_ostream.hpp>
#include <flow/dispatch_ostream.hpp>
#include <flow/captor_state_ostream.hpp>

// Enable streaming for debug
std::cout << "Synchronizer state: " << sync << std::endl;
```

### Common Drop Patterns to Watch

```
1. HIGH ABORT RATE
   ├─ Symptom: Many frames dropped
   ├─ Cause: Mismatched timestamps, missing data
   └─ Fix: Use flexible followers (AnyBefore, ClosestBefore)

2. GROWING QUEUE SIZE
   ├─ Symptom: Memory usage increases
   ├─ Cause: RETRY loops, slow processing
   └─ Fix: Add throttling, increase processing rate

3. UNEXPECTED EMPTY CAPTURES
   ├─ Symptom: Before/AnyBefore return empty
   ├─ Cause: Wrong delay/period configuration
   └─ Fix: Match config to actual data timing

4. PERIODIC ABORTS
   ├─ Symptom: Regular pattern of drops
   ├─ Cause: Timing mismatch in periodic data
   └─ Fix: Adjust period to match actual data rate
```

---

## Summary: Drop Behavior by Captor Type

| Captor | What Gets Dropped | Why |
|--------|-------------------|-----|
| `driver::Next` | Captured element only | One-at-a-time processing |
| `driver::Batch` | Only oldest per capture | Sliding window design |
| `driver::Chunk` | All captured elements | Complete batch processing |
| `driver::Throttled` | Elements within throttle period | Rate limiting by design |
| `follower::Before` | All < boundary | Already captured or stale |
| `follower::ClosestBefore` | All < boundary | Only closest needed |
| `follower::CountBefore` | All < boundary | Only last N needed |
| `follower::Ranged` | Only < lower bound | Bounds kept for context |
| `follower::MatchedStamp` | Captured element only | Exact match required |
| `follower::Latched` | All <= boundary | Only latest needed |
| `follower::AnyBefore` | Implementation-defined | Flexible capture |
| `follower::AnyAtOrBefore` | Implementation-defined | Flexible capture |

### Key Takeaways

1. **Throttled is the main intentional dropper** - use it only when rate limiting is desired
2. **ClosestBefore and CountBefore drop non-selected elements** - older data is discarded
3. **ABORT causes all captors to drop** - synchronization failure loses entire frame
4. **Proof elements are retained** - they become data in subsequent captures
5. **Configure periods/delays carefully** - mismatched config causes unexpected drops
