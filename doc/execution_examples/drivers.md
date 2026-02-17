# Driver Execution Examples

This document provides detailed execution traces for all Flow driver types. Drivers establish the synchronization time range that followers use to select their data.

---

## Table of Contents

1. [driver::Next](#1-drivernext)
2. [driver::Batch](#2-driverbatch)
3. [driver::Chunk](#3-driverchunk)
4. [driver::Throttled](#4-driverthrottled)
5. [Driver Comparison Summary](#5-driver-comparison-summary)

---

## 1. driver::Next

**Purpose:** Captures the single oldest element and establishes a point-in-time range.

**Parameters:** None (default constructor)

**Range Behavior:** `range.lower_stamp == range.upper_stamp == oldest_element.stamp`

**Data Removal:** Removes the captured element only

### Basic Workflow

```cpp
driver::Next<Dispatch<int, string>, NoLock> driver;
```

#### Example 1.1: Simple Sequential Capture

```
INITIAL STATE:
    driver.inject(10, "A")
    driver.inject(20, "B")
    driver.inject(30, "C")
    
    Queue: [10:"A", 20:"B", 30:"C"]
              ↑oldest          ↑newest

CAPTURE 1:
    ├─ locate_driver_impl()
    │   ├─ queue_.empty()? NO
    │   ├─ oldest_stamp() = 10
    │   ├─ range = {lower: 10, upper: 10}
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 1})
    │
    ├─ extract_driver_impl()
    │   ├─ Move queue_[0] to output: Dispatch{10, "A"}
    │   └─ Remove first 1 element
    │
    └─ RESULT: State::PRIMED, range={10, 10}
    
    Output: [Dispatch{10, "A"}]
    Queue After: [20:"B", 30:"C"]
                  ↑oldest   ↑newest

CAPTURE 2:
    ├─ locate_driver_impl()
    │   ├─ oldest_stamp() = 20
    │   ├─ range = {lower: 20, upper: 20}
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 1})
    │
    └─ RESULT: State::PRIMED, range={20, 20}
    
    Output: [Dispatch{20, "B"}]
    Queue After: [30:"C"]

CAPTURE 3:
    └─ RESULT: State::PRIMED, range={30, 30}
    
    Output: [Dispatch{30, "C"}]
    Queue After: [] (empty)

CAPTURE 4:
    ├─ locate_driver_impl()
    │   ├─ queue_.empty()? YES
    │   └─ RETURN: (RETRY, ExtractionRange{})
    │
    └─ RESULT: State::RETRY (no extraction)
```

#### Example 1.2: Out-of-Order Injection

Flow automatically reorders data by timestamp:

```
INJECTION SEQUENCE (out of order):
    driver.inject(30, "C")  // First injection, but highest stamp
    driver.inject(10, "A")  // Second injection, lowest stamp
    driver.inject(20, "B")  // Third injection, middle stamp
    
    Queue (auto-sorted): [10:"A", 20:"B", 30:"C"]
                          ↑oldest          ↑newest

CAPTURE 1:
    └─ Captures stamp=10 (oldest), NOT stamp=30 (first injected)
    
    Output: [Dispatch{10, "A"}]
```

#### Example 1.3: Rapid Injection During Capture

```
TIMELINE:
    t=0: Queue = [10, 20, 30]
    
    CAPTURE starts at t=0:
    ├─ locate() returns range={10, 10}
    │
    │   >>> During extract(), new data arrives <<<
    │   driver.inject(5, "early")   // Earlier timestamp!
    │   driver.inject(40, "late")   // Later timestamp
    │
    ├─ extract() completes, removes stamp=10
    └─ RESULT: range={10, 10}
    
    Queue After: [5, 20, 30, 40]
                  ↑ New oldest! (will be captured next)
    
NEXT CAPTURE:
    └─ Captures stamp=5 (not 20!)
    
    ⚠️ WARNING: This can cause non-monotonic capture if timestamps
    go backwards. Use lower_bound parameter to prevent this.
```

#### Example 1.4: Using lower_bound Parameter

```cpp
// Prevent capturing timestamps before a known point
auto result = Synchronizer::capture(
    std::forward_as_tuple(driver),
    std::forward_as_tuple(output),
    15  // lower_bound: only capture timestamps >= 15
);
```

```
Queue: [10, 20, 30]
lower_bound = 15

CAPTURE:
    ├─ locate() returns range={10, 10}
    ├─ Check: range.upper_stamp(10) < lower_bound(15)?
    │   └─ YES → Set state = ERROR_DRIVER_LOWER_BOUND_EXCEEDED
    └─ RESULT: State::ERROR_DRIVER_LOWER_BOUND_EXCEEDED

⚠️ This is a CRITICAL ERROR indicating timestamp monotonicity violation
```

---

## 2. driver::Batch

**Purpose:** Captures N oldest elements, creating a sliding window effect.

**Parameters:** `size_type batch_size` (must be > 0)

**Range Behavior:** `range.lower_stamp = oldest.stamp`, `range.upper_stamp = newest_captured.stamp`

**Data Removal:** Removes ONLY the oldest element (enables overlap)

### Basic Workflow

```cpp
driver::Batch<Dispatch<int, double>, NoLock> driver{3}; // batch_size = 3
```

#### Example 2.1: Sliding Window Effect

```
SETUP:
    batch_size = 3
    
    for (int t = 0; t < 10; t += 2) {
        driver.inject(t, t * 0.5);  // stamps: 0, 2, 4, 6, 8
    }
    
    Queue: [0, 2, 4, 6, 8]

CAPTURE 1:
    ├─ locate_driver_impl()
    │   ├─ queue_.size() = 5 >= batch_size(3)? YES
    │   ├─ range.lower_stamp = queue_[0].stamp = 0
    │   ├─ range.upper_stamp = queue_[2].stamp = 4
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 3})
    │
    ├─ extract_driver_impl()
    │   ├─ Move queue_[0..2] to output: stamps 0, 2, 4
    │   └─ Remove first 1 element only (sliding window!)
    │
    └─ RESULT: State::PRIMED, range={0, 4}
    
    Output: [Dispatch{0}, Dispatch{2}, Dispatch{4}]
    Queue After: [2, 4, 6, 8]  ← stamp 0 removed, others remain
                  ↑ new oldest

CAPTURE 2:
    ├─ locate_driver_impl()
    │   ├─ range = {lower: 2, upper: 6}
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 3})
    │
    └─ RESULT: State::PRIMED, range={2, 6}
    
    Output: [Dispatch{2}, Dispatch{4}, Dispatch{6}]  ← OVERLAPPING!
    Queue After: [4, 6, 8]
    
    Notice: Dispatch{2} and Dispatch{4} captured AGAIN (overlap)

CAPTURE 3:
    └─ RESULT: State::PRIMED, range={4, 8}
    Output: [Dispatch{4}, Dispatch{6}, Dispatch{8}]
    Queue After: [6, 8]  ← Only 2 elements remain

CAPTURE 4:
    ├─ locate_driver_impl()
    │   ├─ queue_.size() = 2 < batch_size(3)? YES
    │   └─ RETURN: (RETRY, ExtractionRange{})
    │
    └─ RESULT: State::RETRY (need 1 more element)
```

#### Example 2.2: Batch Size Edge Cases

```
CASE A: batch_size = 1 (equivalent to Next)
    Queue: [10, 20, 30]
    Capture 1: Output=[10], Queue After=[20, 30]
    Capture 2: Output=[20], Queue After=[30]
    → Behaves exactly like driver::Next

CASE B: batch_size = queue.size() (full capture)
    batch_size = 5
    Queue: [10, 20, 30, 40, 50]
    
    Capture 1: 
        Output=[10, 20, 30, 40, 50], range={10, 50}
        Queue After=[20, 30, 40, 50]  ← Only oldest removed
    
    Capture 2:
        queue.size() = 4 < 5 → RETRY

CASE C: batch_size = 0 (INVALID)
    driver::Batch<...> driver{0};
    → throws std::invalid_argument
```

#### Example 2.3: Use Case - Moving Average

```cpp
// Calculate moving average over 5 samples
driver::Batch<Dispatch<int, double>, NoLock> driver{5};

// Inject sensor readings
for (int t = 0; t < 100; ++t) {
    driver.inject(t, read_sensor());
}

while (true) {
    std::vector<Dispatch<int, double>> window;
    auto result = Synchronizer::capture(
        std::forward_as_tuple(driver),
        std::forward_as_tuple(std::back_inserter(window))
    );
    
    if (result.state == State::PRIMED) {
        // window contains 5 overlapping samples
        double sum = 0;
        for (auto& d : window) sum += d.value;
        double avg = sum / 5.0;
        
        // Next capture will shift window by 1
    }
}
```

---

## 3. driver::Chunk

**Purpose:** Captures N oldest elements as a non-overlapping batch.

**Parameters:** `size_type chunk_size` (must be > 0)

**Range Behavior:** Same as Batch

**Data Removal:** Removes ALL captured elements (no overlap)

### Basic Workflow

```cpp
driver::Chunk<Dispatch<int, string>, NoLock> driver{3}; // chunk_size = 3
```

#### Example 3.1: Non-Overlapping Batches

```
SETUP:
    chunk_size = 3
    Queue: [0, 1, 2, 3, 4, 5, 6, 7, 8]

CAPTURE 1:
    ├─ locate_driver_impl()
    │   ├─ queue_.size() = 9 >= chunk_size(3)? YES
    │   ├─ range = {lower: 0, upper: 2}
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 3})
    │
    ├─ extract_driver_impl()
    │   ├─ Move queue_[0..2] to output
    │   └─ Remove first 3 elements (ALL captured)
    │
    └─ RESULT: State::PRIMED, range={0, 2}
    
    Output: [0, 1, 2]
    Queue After: [3, 4, 5, 6, 7, 8]  ← Clean cut!

CAPTURE 2:
    └─ RESULT: State::PRIMED, range={3, 5}
    
    Output: [3, 4, 5]  ← NO OVERLAP with previous
    Queue After: [6, 7, 8]

CAPTURE 3:
    └─ RESULT: State::PRIMED, range={6, 8}
    
    Output: [6, 7, 8]
    Queue After: []

CAPTURE 4:
    └─ RESULT: State::RETRY (empty queue)
```

#### Example 3.2: Batch vs Chunk Comparison

```
SAME DATA: Queue = [0, 1, 2, 3, 4, 5], size = 3

┌─────────────────────────────────────────────────────────┐
│                    driver::Batch{3}                     │
├─────────────────────────────────────────────────────────┤
│ Capture 1: Output=[0,1,2]  Queue After=[1,2,3,4,5]     │
│ Capture 2: Output=[1,2,3]  Queue After=[2,3,4,5]       │
│ Capture 3: Output=[2,3,4]  Queue After=[3,4,5]         │
│ Capture 4: Output=[3,4,5]  Queue After=[4,5]           │
│ Capture 5: RETRY (only 2 elements)                      │
│                                                         │
│ Total captures: 4 (with overlap)                        │
│ Element 2 captured: 3 times                             │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│                    driver::Chunk{3}                     │
├─────────────────────────────────────────────────────────┤
│ Capture 1: Output=[0,1,2]  Queue After=[3,4,5]         │
│ Capture 2: Output=[3,4,5]  Queue After=[]              │
│ Capture 3: RETRY (empty)                                │
│                                                         │
│ Total captures: 2 (no overlap)                          │
│ Element 2 captured: 1 time                              │
└─────────────────────────────────────────────────────────┘
```

#### Example 3.3: Use Case - Packet Processing

```cpp
// Process network packets in fixed-size batches
driver::Chunk<Dispatch<int, Packet>, NoLock> driver{100}; // 100 packets per batch

while (receiving) {
    driver.inject(sequence_num, packet);
}

// Process complete batches
while (true) {
    std::vector<Dispatch<int, Packet>> batch;
    auto result = Synchronizer::capture(
        std::forward_as_tuple(driver),
        std::forward_as_tuple(std::back_inserter(batch))
    );
    
    if (result.state == State::PRIMED) {
        // Process exactly 100 packets
        // No packet processed twice
        process_batch(batch);
    } else {
        break; // Wait for more packets
    }
}
```

---

## 4. driver::Throttled

**Purpose:** Rate-limited capture that skips elements to maintain a maximum capture rate.

**Parameters:** `offset_type throttle_period` (minimum time between captures)

**Range Behavior:** `range.lower_stamp == range.upper_stamp == captured_element.stamp`

**Data Removal:** Removes captured element AND all older elements

### Basic Workflow

```cpp
driver::Throttled<Dispatch<int, double>, NoLock> driver{10}; // throttle_period = 10
```

#### Example 4.1: Throttling High-Frequency Data

```
SETUP:
    throttle_period = 10
    
    // High-frequency injection (every 2 time units)
    for (int t = 0; t <= 30; t += 2) {
        driver.inject(t, t * 1.0);  // stamps: 0, 2, 4, 6, 8, 10, 12, 14, ...
    }
    
    Queue: [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]
    
    Internal state: last_captured_stamp_ = min (no previous capture)

CAPTURE 1:
    ├─ locate_driver_impl()
    │   ├─ last_captured_stamp_ = min (first capture)
    │   ├─ target = min + throttle_period ≈ min
    │   ├─ Find first element >= target → stamp 0
    │   ├─ range = {0, 0}
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 1})
    │
    ├─ extract_driver_impl()
    │   ├─ Move stamp 0 to output
    │   ├─ Update last_captured_stamp_ = 0
    │   └─ Remove all elements <= 0 (just stamp 0)
    │
    └─ RESULT: State::PRIMED, range={0, 0}
    
    Output: [Dispatch{0, 0.0}]
    Queue After: [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]

CAPTURE 2:
    ├─ locate_driver_impl()
    │   ├─ last_captured_stamp_ = 0
    │   ├─ target = 0 + 10 = 10
    │   ├─ Find first element >= 10 → stamp 10
    │   ├─ Skip stamps 2, 4, 6, 8 (too early)
    │   ├─ range = {10, 10}
    │   └─ RETURN: (PRIMED, ExtractionRange for stamp 10)
    │
    ├─ extract_driver_impl()
    │   ├─ Move stamp 10 to output
    │   ├─ Update last_captured_stamp_ = 10
    │   └─ Remove all elements <= 10 (stamps 2,4,6,8,10)
    │
    └─ RESULT: State::PRIMED, range={10, 10}
    
    Output: [Dispatch{10, 10.0}]
    Queue After: [12, 14, 16, 18, 20, 22, 24, 26, 28, 30]
    
    ⚠️ DROPPED: stamps 2, 4, 6, 8 (within throttle period)

CAPTURE 3:
    ├─ target = 10 + 10 = 20
    ├─ Find first element >= 20 → stamp 20
    ├─ Skip stamps 12, 14, 16, 18
    └─ RESULT: State::PRIMED, range={20, 20}
    
    Output: [Dispatch{20, 20.0}]
    Queue After: [22, 24, 26, 28, 30]
    
    ⚠️ DROPPED: stamps 12, 14, 16, 18

CAPTURE 4:
    ├─ target = 20 + 10 = 30
    ├─ Find first element >= 30 → stamp 30
    └─ RESULT: State::PRIMED, range={30, 30}
    
    Output: [Dispatch{30, 30.0}]
    Queue After: []
    
    ⚠️ DROPPED: stamps 22, 24, 26, 28
```

#### Example 4.2: Throttle Period > Data Span

```
throttle_period = 100
Queue: [0, 2, 4, 6, 8]

CAPTURE 1:
    ├─ target = min + 100 ≈ min
    ├─ Find first >= min → stamp 0
    └─ RESULT: State::PRIMED, range={0, 0}
    
    Queue After: [2, 4, 6, 8]

CAPTURE 2:
    ├─ target = 0 + 100 = 100
    ├─ Find first >= 100 → NONE FOUND
    │   (newest stamp is 8 < 100)
    └─ RESULT: State::RETRY
    
    Queue After: [2, 4, 6, 8] (unchanged)
    
    → Need to wait for data with stamp >= 100
```

#### Example 4.3: Use Case - Downsampling Camera Stream

```cpp
// Camera at 60fps, process at 10fps
// throttle_period = 100ms (for 10fps)

using TimePoint = std::chrono::steady_clock::time_point;
using Duration = std::chrono::milliseconds;
using CameraDispatch = Dispatch<TimePoint, cv::Mat>;

driver::Throttled<CameraDispatch, NoLock> driver{Duration{100}};

// Camera callback (60fps)
void on_frame(const cv::Mat& frame) {
    driver.inject(std::chrono::steady_clock::now(), frame);
}

// Processing loop (will run at ~10fps)
while (running) {
    std::vector<CameraDispatch> frames;
    auto result = Synchronizer::capture(...);
    
    if (result.state == State::PRIMED) {
        // Process ~10 frames per second
        // ~5 frames dropped between each capture
        process_frame(frames[0].value);
    }
}
```

---

## 5. Driver Comparison Summary

| Driver | Captures | Removes | Overlap | Use Case |
|--------|----------|---------|---------|----------|
| `Next` | 1 oldest | 1 captured | No | Frame-by-frame |
| `Batch{N}` | N oldest | 1 oldest | Yes | Sliding window |
| `Chunk{N}` | N oldest | N captured | No | Fixed batches |
| `Throttled{P}` | 1 at interval | All before | No (skips) | Rate limiting |

### Decision Tree

```
Need to capture multiple elements per sync?
├─ NO → driver::Next
│
└─ YES → Need overlap between captures?
         ├─ YES → driver::Batch
         │
         └─ NO → Need rate limiting?
                 ├─ YES → driver::Throttled
                 │        (will drop intermediate data)
                 │
                 └─ NO → driver::Chunk
```

### Range Visualization

```
Data timeline: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

driver::Next:
    Cap1: [0]─────────────────────────────────  range={0,0}
    Cap2:    [1]──────────────────────────────  range={1,1}
    Cap3:       [2]───────────────────────────  range={2,2}

driver::Batch{4}:
    Cap1: [0, 1, 2, 3]────────────────────────  range={0,3}
    Cap2:    [1, 2, 3, 4]─────────────────────  range={1,4}  ← overlap
    Cap3:       [2, 3, 4, 5]──────────────────  range={2,5}  ← overlap

driver::Chunk{4}:
    Cap1: [0, 1, 2, 3]────────────────────────  range={0,3}
    Cap2:             [4, 5, 6, 7]────────────  range={4,7}  ← no overlap
    Cap3:                         [8, 9, ...]  range={8,...}

driver::Throttled{5}:
    Cap1: [0]─────────────────────────────────  range={0,0}
    Cap2:                [5]──────────────────  range={5,5}  ← skipped 1-4
    Cap3:                               [10]─  range={10,10} ← skipped 6-9
```
