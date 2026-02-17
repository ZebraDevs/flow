# Follower Execution Examples

This document provides detailed execution traces for all Flow follower types. Followers select data based on the synchronization range established by a driver.

---

## Table of Contents

1. [follower::Before](#1-followerbefore)
2. [follower::ClosestBefore](#2-followerclosestbefore)
3. [follower::CountBefore](#3-followercountbefore)
4. [follower::Ranged](#4-followerranged)
5. [follower::MatchedStamp](#5-followermatchedstamp)
6. [follower::Latched](#6-followerlatched)
7. [follower::AnyBefore](#7-followeranybefore)
8. [follower::AnyAtOrBefore](#8-followeranyatorbefore)
9. [Follower Comparison Summary](#9-follower-comparison-summary)

---

## 1. follower::Before

**Purpose:** Captures ALL elements before the synchronization boundary.

**Parameters:** `offset_type delay` (offset from driver's range)

**Boundary:** `range.upper_stamp - delay` (non-inclusive)

**Requirements:** At least one element AFTER boundary must exist (proves data is complete)

**Data Removal:** All captured elements removed

### Basic Workflow

```cpp
follower::Before<Dispatch<int, string>, NoLock> follower{5}; // delay = 5
```

#### Example 1.1: Standard Capture

```
SETUP:
    delay = 5
    Driver establishes range = {20, 20}
    
    Follower queue: [5, 8, 10, 12, 15, 18, 22, 25]
                     ↑oldest                  ↑newest

CAPTURE:
    ├─ locate_follower_impl(range={20, 20})
    │   ├─ boundary = range.upper_stamp - delay = 20 - 5 = 15
    │   │
    │   ├─ Check: queue_.empty()? NO
    │   ├─ Check: newest_stamp(25) >= boundary(15)? YES
    │   │         (proves we have data after boundary)
    │   │
    │   ├─ Iterate: collect stamps < 15
    │   │   ├─ stamp 5 < 15 → INCLUDE
    │   │   ├─ stamp 8 < 15 → INCLUDE
    │   │   ├─ stamp 10 < 15 → INCLUDE
    │   │   ├─ stamp 12 < 15 → INCLUDE
    │   │   └─ stamp 15 >= 15 → STOP
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 4})
    │
    ├─ extract_follower_impl()
    │   ├─ Move elements [0..3] to output: stamps 5, 8, 10, 12
    │   └─ Remove first 4 elements
    │
    └─ RESULT: State::PRIMED
    
    Output: [Dispatch{5}, Dispatch{8}, Dispatch{10}, Dispatch{12}]
    Queue After: [15, 18, 22, 25]
```

#### Example 1.2: RETRY - No Data After Boundary

```
SETUP:
    delay = 5
    Driver range = {20, 20}
    boundary = 15
    
    Follower queue: [5, 8, 10, 12]  ← All before boundary, nothing after!

CAPTURE:
    ├─ locate_follower_impl(range={20, 20})
    │   ├─ boundary = 15
    │   ├─ newest_stamp(12) >= boundary(15)? NO!
    │   │   └─ Cannot prove data before boundary is complete
    │   │
    │   └─ RETURN: (RETRY, ExtractionRange{})
    │
    └─ RESULT: State::RETRY (no extraction)
    
    Queue After: [5, 8, 10, 12] (unchanged)
    
    → Need data with stamp >= 15 to prove completeness
```

#### Example 1.3: Empty Capture (All Data After Boundary)

```
SETUP:
    delay = 5
    Driver range = {20, 20}
    boundary = 15
    
    Follower queue: [16, 18, 22, 25]  ← All >= 15!

CAPTURE:
    ├─ locate_follower_impl()
    │   ├─ boundary = 15
    │   ├─ newest_stamp(25) >= 15? YES (have proof)
    │   ├─ Iterate: collect stamps < 15
    │   │   └─ stamp 16 >= 15 → STOP (nothing to capture)
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 0})  ← Empty range!
    │
    └─ RESULT: State::PRIMED (with empty output)
    
    Output: [] (empty, but valid!)
    Queue After: [16, 18, 22, 25] (unchanged)
```

#### Example 1.4: Large Delay Effect

```
SETUP:
    delay = 100  ← Large delay
    Driver range = {50, 50}
    boundary = 50 - 100 = -50  ← Negative boundary!
    
    Follower queue: [10, 20, 30, 40, 60]

CAPTURE:
    ├─ boundary = -50
    ├─ Iterate: collect stamps < -50
    │   └─ stamp 10 >= -50 → STOP (nothing before -50)
    │
    └─ RESULT: State::PRIMED (empty output)
    
    Output: [] (no data before t=-50)
    
    ⚠️ With large delays, you may capture nothing!
```

---

## 2. follower::ClosestBefore

**Purpose:** Captures the SINGLE element closest to (but before) the boundary.

**Parameters:** 
- `offset_type period` - Expected data period (search window size)
- `offset_type delay` - Offset from driver's range

**Boundary:** `range.lower_stamp - delay`

**Search Window:** `[boundary - period, boundary)`

**Data Removal:** All elements before the captured one (inclusive)

### Basic Workflow

```cpp
follower::ClosestBefore<Dispatch<int, double>, NoLock> follower{
    10,  // period: expect data every ~10 time units
    5    // delay: look 5 time units before driver
};
```

#### Example 2.1: Standard Capture

```
SETUP:
    period = 10, delay = 5
    Driver range = {100, 100}
    
    boundary = 100 - 5 = 95
    search_window = [95 - 10, 95) = [85, 95)
    
    Follower queue: [70, 80, 88, 92, 98, 105]

CAPTURE:
    ├─ locate_follower_impl(range={100, 100})
    │   ├─ boundary = 95
    │   ├─ search_window = [85, 95)
    │   │
    │   ├─ Check: Have element >= boundary? 
    │   │   └─ stamp 98 >= 95? YES (proves search is complete)
    │   │
    │   ├─ Find closest in window [85, 95):
    │   │   ├─ stamp 70: not in [85, 95)
    │   │   ├─ stamp 80: not in [85, 95)
    │   │   ├─ stamp 88: IN [85, 95) ✓
    │   │   ├─ stamp 92: IN [85, 95) ✓ ← CLOSEST to 95
    │   │   └─ stamp 98: not in [85, 95)
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange pointing to stamp 92)
    │
    ├─ extract_follower_impl()
    │   ├─ Move stamp 92 to output
    │   └─ Remove stamps 70, 80, 88, 92 (all up to captured)
    │
    └─ RESULT: State::PRIMED
    
    Output: [Dispatch{92, ...}]
    Queue After: [98, 105]
```

#### Example 2.2: ABORT - No Element in Window

```
SETUP:
    period = 10, delay = 5
    Driver range = {100, 100}
    search_window = [85, 95)
    
    Follower queue: [70, 80, 98, 105]  ← Gap! No data in [85, 95)

CAPTURE:
    ├─ locate_follower_impl()
    │   ├─ boundary = 95, window = [85, 95)
    │   ├─ Have element >= 95? YES (stamp 98)
    │   ├─ Find closest in [85, 95):
    │   │   ├─ stamp 70: not in window
    │   │   ├─ stamp 80: not in window (80 < 85)
    │   │   ├─ stamp 98: not in window (98 >= 95)
    │   │   └─ NO ELEMENT FOUND IN WINDOW
    │   │
    │   └─ RETURN: (ABORT, ExtractionRange{})
    │
    └─ RESULT: State::ABORT
    
    ⚠️ ABORT indicates the data gap is permanent - no point waiting
    The synchronization frame is skipped/dropped
```

#### Example 2.3: RETRY - No Proof of Completeness

```
SETUP:
    period = 10, delay = 5
    Driver range = {100, 100}
    search_window = [85, 95)
    
    Follower queue: [70, 80, 88, 92]  ← No data >= 95!

CAPTURE:
    ├─ locate_follower_impl()
    │   ├─ boundary = 95, window = [85, 95)
    │   ├─ Have element >= 95? NO (newest is 92)
    │   │   └─ Cannot confirm 92 is the closest
    │   │      (data with stamp 94 might arrive!)
    │   │
    │   └─ RETURN: (RETRY, ExtractionRange{})
    │
    └─ RESULT: State::RETRY
    
    Queue After: [70, 80, 88, 92] (unchanged)
    
    → Need to wait for data with stamp >= 95
```

#### Example 2.4: Period Misconfiguration Warning

```
SETUP:
    period = 5  ← TOO SMALL! Actual data period is ~20
    delay = 0
    Driver range = {100, 100}
    search_window = [95, 100)  ← Very narrow!
    
    Follower queue: [60, 80, 105]  ← Data every ~20-25 units

CAPTURE:
    ├─ Find in [95, 100):
    │   ├─ stamp 60: not in window
    │   ├─ stamp 80: not in window (< 95)
    │   └─ stamp 105: not in window (>= 100)
    │
    └─ RESULT: State::ABORT (missed stamp 80!)
    
    ⚠️ WARNING: Period too small causes unnecessary ABORTs!
    Solution: Set period >= actual_data_period
```

---

## 3. follower::CountBefore

**Purpose:** Captures exactly N elements before the boundary.

**Parameters:**
- `size_type count` - Number of elements to capture
- `offset_type delay` - Offset from driver's range

**Boundary:** `range.lower_stamp - delay`

**Data Removal:** All elements before the N-th captured element

### Basic Workflow

```cpp
follower::CountBefore<Dispatch<int, double>, NoLock> follower{
    3,   // count: capture exactly 3 elements
    5    // delay: look 5 time units before driver
};
```

#### Example 3.1: Standard Capture

```
SETUP:
    count = 3, delay = 5
    Driver range = {100, 100}
    boundary = 95
    
    Follower queue: [70, 80, 85, 90, 98, 105]

CAPTURE:
    ├─ locate_follower_impl(range={100, 100})
    │   ├─ boundary = 95
    │   │
    │   ├─ Count elements < 95:
    │   │   stamps 70, 80, 85, 90 → 4 elements < 95
    │   │
    │   ├─ Have at least count(3) elements? YES
    │   ├─ Have element >= 95 (proof)? YES (stamp 98)
    │   │
    │   ├─ Select last 3 before boundary: 80, 85, 90
    │   │   (not 70 - it's the 4th oldest)
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange for 80, 85, 90)
    │
    ├─ extract_follower_impl()
    │   ├─ Move stamps 80, 85, 90 to output
    │   └─ Remove stamps 70, 80, 85, 90 (including older)
    │
    └─ RESULT: State::PRIMED
    
    Output: [Dispatch{80}, Dispatch{85}, Dispatch{90}]
    Queue After: [98, 105]
```

#### Example 3.2: RETRY - Not Enough Elements

```
SETUP:
    count = 5, delay = 5
    Driver range = {100, 100}
    boundary = 95
    
    Follower queue: [80, 85, 90, 98]  ← Only 3 elements < 95!

CAPTURE:
    ├─ locate_follower_impl()
    │   ├─ boundary = 95
    │   ├─ Elements < 95: stamps 80, 85, 90 (only 3)
    │   ├─ Have count(5) elements? NO (only 3)
    │   │
    │   └─ RETURN: (RETRY, ExtractionRange{})
    │
    └─ RESULT: State::RETRY
    
    → Need 2 more elements with stamp < 95
    
    ⚠️ This might wait forever if no more old data arrives!
```

#### Example 3.3: ABORT - Data Gap

```
SETUP:
    count = 3, delay = 5
    Driver range = {100, 100}
    boundary = 95
    
    Follower queue: [98, 105, 110]  ← All elements >= 95!

CAPTURE:
    ├─ locate_follower_impl()
    │   ├─ boundary = 95
    │   ├─ Elements < 95: NONE
    │   ├─ Have element >= 95? YES
    │   │   └─ Proof that no old data will arrive
    │   │
    │   └─ RETURN: (ABORT, ExtractionRange{})
    │
    └─ RESULT: State::ABORT
    
    ⚠️ Cannot fulfill count requirement - frame dropped
```

---

## 4. follower::Ranged

**Purpose:** Captures elements spanning the ENTIRE driver range, plus one before and one after.

**Parameters:** `offset_type delay` - Offset from driver's range

**Capture:** Elements from `(range.lower_stamp - delay)` to `(range.upper_stamp - delay)` inclusive, plus boundaries

**Data Removal:** All elements up to and including the "before" boundary element

### Basic Workflow

```cpp
follower::Ranged<Dispatch<int, double>, NoLock> follower{0}; // delay = 0
```

#### Example 4.1: Standard Capture with Range Span

```
SETUP:
    delay = 0
    Driver range = {100, 120}  ← Range spans 100 to 120
    
    Follower queue: [80, 90, 95, 105, 110, 115, 125, 130]

CAPTURE:
    ├─ locate_follower_impl(range={100, 120})
    │   ├─ Need:
    │   │   ├─ 1 element BEFORE 100 (boundary anchor)
    │   │   ├─ All elements IN [100, 120]
    │   │   └─ 1 element AFTER 120 (boundary anchor)
    │   │
    │   ├─ Find before 100: stamp 95 (closest < 100)
    │   ├─ Find after 120: stamp 125 (closest > 120)
    │   ├─ Elements in range: 105, 110, 115
    │   │
    │   ├─ Capture set: [95, 105, 110, 115, 125]
    │   │   (before, in-range, after)
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange)
    │
    ├─ extract_follower_impl()
    │   ├─ Move [95, 105, 110, 115, 125] to output
    │   └─ Remove [80, 90, 95] (up to and including "before")
    │
    └─ RESULT: State::PRIMED
    
    Output: [Dispatch{95}, Dispatch{105}, Dispatch{110}, 
             Dispatch{115}, Dispatch{125}]
    Queue After: [105, 110, 115, 125, 130]
    
    Note: 105, 110, 115, 125 remain (might be needed for next capture)
```

#### Example 4.2: Point Range (lower == upper)

```
SETUP:
    delay = 0
    Driver range = {100, 100}  ← Point in time (from driver::Next)
    
    Follower queue: [80, 90, 95, 100, 105, 110]

CAPTURE:
    ├─ Need:
    │   ├─ 1 before 100: stamp 95
    │   ├─ Elements in [100, 100]: stamp 100 (if exists)
    │   └─ 1 after 100: stamp 105
    │
    ├─ Capture: [95, 100, 105]
    │   (If stamp 100 doesn't exist, still captures [95, 105])
    │
    └─ RESULT: State::PRIMED
    
    Output: [Dispatch{95}, Dispatch{100}, Dispatch{105}]
```

#### Example 4.3: ABORT - No Element Before

```
SETUP:
    delay = 0
    Driver range = {100, 120}
    
    Follower queue: [105, 110, 115, 125]  ← Nothing < 100!

CAPTURE:
    ├─ Find before 100: NOT FOUND
    │   └─ Cannot anchor the range start
    │
    └─ RESULT: State::ABORT
    
    ⚠️ ABORT because we can't guarantee data before the range
```

#### Example 4.4: RETRY - No Element After

```
SETUP:
    delay = 0
    Driver range = {100, 120}
    
    Follower queue: [80, 90, 95, 105, 110, 115]  ← Nothing > 120!

CAPTURE:
    ├─ Find before 100: stamp 95 ✓
    ├─ Find after 120: NOT FOUND
    │   └─ Cannot confirm range end is complete
    │
    └─ RESULT: State::RETRY
    
    → Need data with stamp > 120
```

---

## 5. follower::MatchedStamp

**Purpose:** Captures element with EXACT timestamp match to driver's range.

**Parameters:** None (default constructor)

**Requirement:** `element.stamp == range.lower_stamp`

**Data Removal:** All elements up to and including matched element

### Basic Workflow

```cpp
follower::MatchedStamp<Dispatch<int, string>, NoLock> follower;
```

#### Example 5.1: Exact Match Found

```
SETUP:
    Driver range = {100, 100}
    
    Follower queue: [80, 90, 100, 110, 120]
                              ↑ Exact match!

CAPTURE:
    ├─ locate_follower_impl(range={100, 100})
    │   ├─ Search for stamp == 100
    │   ├─ Found: stamp 100 exists!
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange for stamp 100)
    │
    ├─ extract_follower_impl()
    │   ├─ Move stamp 100 to output
    │   └─ Remove stamps 80, 90, 100
    │
    └─ RESULT: State::PRIMED
    
    Output: [Dispatch{100, ...}]
    Queue After: [110, 120]
```

#### Example 5.2: ABORT - Stamp Missed (Gap)

```
SETUP:
    Driver range = {100, 100}
    
    Follower queue: [80, 90, 105, 110]  ← No stamp 100!
                              ↑ Jumped past 100

CAPTURE:
    ├─ locate_follower_impl(range={100, 100})
    │   ├─ Search for stamp == 100
    │   ├─ Found stamp > 100 (stamp 105)
    │   │   └─ Proves stamp 100 will never arrive
    │   │
    │   └─ RETURN: (ABORT, ExtractionRange{})
    │
    └─ RESULT: State::ABORT
    
    Queue After: [80, 90, 105, 110] (unchanged until abort cleanup)
    
    ⚠️ ABORT: exact match impossible, frame dropped
```

#### Example 5.3: RETRY - Not Yet Available

```
SETUP:
    Driver range = {100, 100}
    
    Follower queue: [80, 90, 95]  ← All < 100

CAPTURE:
    ├─ locate_follower_impl(range={100, 100})
    │   ├─ Search for stamp == 100
    │   ├─ newest_stamp(95) < 100
    │   │   └─ Stamp 100 might still arrive
    │   │
    │   └─ RETURN: (RETRY, ExtractionRange{})
    │
    └─ RESULT: State::RETRY
    
    → Wait for stamp 100 to arrive
```

---

## 6. follower::Latched

**Purpose:** Captures and HOLDS the last valid element, returning it on subsequent captures.

**Parameters:** `offset_type min_period` - Minimum time between data updates

**Behavior:** 
- First capture: waits for data
- Subsequent: returns latched value OR updates if newer data available

**Data Removal:** All elements before latched element

### Basic Workflow

```cpp
follower::Latched<Dispatch<int, Config>, NoLock> follower{50}; // min_period = 50
```

#### Example 6.1: Initial Latch and Retention

```
SETUP:
    min_period = 50
    
    // Sparse data injection
    follower.inject(0, "config_v1");
    follower.inject(100, "config_v2");
    follower.inject(200, "config_v3");
    
    Follower queue: [0:"v1", 100:"v2", 200:"v3"]
    latched_ = nullopt (nothing latched yet)

CAPTURE 1 with Driver range = {60, 60}:
    ├─ locate_follower_impl(range={60, 60})
    │   ├─ boundary = 60 - 50 = 10
    │   ├─ Find element with stamp <= 10
    │   │   └─ stamp 0 <= 10 ✓
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange for stamp 0)
    │
    ├─ extract_follower_impl()
    │   ├─ Move stamp 0 to output
    │   ├─ Set latched_ = Dispatch{0, "config_v1"}
    │   └─ Remove stamp 0
    │
    └─ RESULT: State::PRIMED
    
    Output: [Dispatch{0, "config_v1"}]
    latched_ = Dispatch{0, "config_v1"}
    Queue After: [100:"v2", 200:"v3"]

CAPTURE 2 with Driver range = {80, 80}:
    ├─ locate_follower_impl(range={80, 80})
    │   ├─ boundary = 80 - 50 = 30
    │   ├─ Find element with stamp <= 30
    │   │   └─ NONE in queue (oldest is 100)
    │   ├─ But latched_ has value!
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 0})  ← Use latched!
    │
    ├─ extract_follower_impl()
    │   └─ Copy latched_ to output (no queue modification)
    │
    └─ RESULT: State::PRIMED
    
    Output: [Dispatch{0, "config_v1"}]  ← Same as before!
    Queue After: [100:"v2", 200:"v3"] (unchanged)

CAPTURE 3 with Driver range = {160, 160}:
    ├─ locate_follower_impl(range={160, 160})
    │   ├─ boundary = 160 - 50 = 110
    │   ├─ Find element with stamp <= 110
    │   │   └─ stamp 100 <= 110 ✓ (NEW DATA!)
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange for stamp 100)
    │
    ├─ extract_follower_impl()
    │   ├─ Move stamp 100 to output
    │   ├─ Update latched_ = Dispatch{100, "config_v2"}
    │   └─ Remove stamp 100
    │
    └─ RESULT: State::PRIMED
    
    Output: [Dispatch{100, "config_v2"}]  ← Updated!
    latched_ = Dispatch{100, "config_v2"}
    Queue After: [200:"v3"]
```

#### Example 6.2: RETRY - No Initial Data

```
SETUP:
    min_period = 50
    latched_ = nullopt
    Follower queue: []  ← Empty!

CAPTURE with Driver range = {100, 100}:
    ├─ locate_follower_impl()
    │   ├─ queue_.empty()? YES
    │   ├─ latched_.has_value()? NO
    │   │
    │   └─ RETURN: (RETRY, ExtractionRange{})
    │
    └─ RESULT: State::RETRY
    
    ⚠️ Latched needs at least one initial value!
```

#### Example 6.3: ABORT - All Data Too New

```
SETUP:
    min_period = 50
    latched_ = nullopt
    Follower queue: [200:"v3", 300:"v4"]  ← All stamps far in future
    
CAPTURE with Driver range = {100, 100}:
    ├─ boundary = 100 - 50 = 50
    ├─ Find stamp <= 50: NONE (oldest is 200)
    ├─ latched_.has_value()? NO
    ├─ Have element > boundary? YES (200 > 50)
    │   └─ Proves no older data will arrive
    │
    └─ RESULT: State::ABORT
```

#### Example 6.4: Reset Clears Latch

```
CAPTURE SEQUENCE:
    1. Capture → latched_ = Dispatch{100, "v2"}
    2. Capture → uses latched (no new data)
    3. follower.reset()  ← RESET CALLED
       └─ latched_ = nullopt
    4. Capture → RETRY (no latched value!)
```

---

## 7. follower::AnyBefore

**Purpose:** Optionally captures ALL elements before boundary. ALWAYS returns PRIMED.

**Parameters:** `offset_type delay` - Offset from driver's range

**Behavior:** Captures whatever is available, including nothing

**Data Removal:** All elements before boundary

### Basic Workflow

```cpp
follower::AnyBefore<Dispatch<int, Event>, NoLock> follower{5}; // delay = 5
```

#### Example 7.1: Capture Available Data

```
SETUP:
    delay = 5
    Driver range = {100, 100}
    boundary = 95
    
    Follower queue: [80, 85, 90, 98, 105]

CAPTURE:
    ├─ locate_follower_impl(range={100, 100})
    │   ├─ boundary = 95
    │   ├─ Collect all stamps < 95: [80, 85, 90]
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 3})  ← Always PRIMED!
    │
    └─ RESULT: State::PRIMED
    
    Output: [Dispatch{80}, Dispatch{85}, Dispatch{90}]
    Queue After: [98, 105]
```

#### Example 7.2: Empty Queue - Still PRIMED

```
SETUP:
    delay = 5
    Driver range = {100, 100}
    
    Follower queue: []  ← Empty!

CAPTURE:
    ├─ locate_follower_impl()
    │   ├─ boundary = 95
    │   ├─ queue_.empty()? YES
    │   │
    │   └─ RETURN: (PRIMED, ExtractionRange{0, 0})  ← Still PRIMED!
    │
    └─ RESULT: State::PRIMED
    
    Output: []  ← Empty but valid
    
    ✓ Perfect for OPTIONAL data streams!
```

#### Example 7.3: All Data After Boundary - Still PRIMED

```
SETUP:
    delay = 5
    Driver range = {100, 100}
    boundary = 95
    
    Follower queue: [98, 105, 110]  ← All >= 95

CAPTURE:
    ├─ Collect stamps < 95: NONE
    │
    └─ RESULT: State::PRIMED
    
    Output: []  ← Empty but valid
    Queue After: [98, 105, 110] (unchanged)
```

#### Example 7.4: Non-Determinism Warning

```
⚠️ WARNING: AnyBefore can behave non-deterministically!

SCENARIO A - Fast data arrival:
    t=0:  Driver range = {100, 100}
    t=0:  Follower queue = [80, 85, 90]
    t=0:  Capture → Output: [80, 85, 90]

SCENARIO B - Slow data arrival (same logical time):
    t=0:  Driver range = {100, 100}
    t=0:  Follower queue = [80, 85]  ← 90 not arrived yet!
    t=0:  Capture → Output: [80, 85]  ← DIFFERENT RESULT!
    
    Later: stamp 90 arrives but boundary already passed
           → stamp 90 will be captured in NEXT sync frame
           
SOLUTION: Use appropriate delay to allow data to arrive
          Or use follower::Before for deterministic behavior
```

---

## 8. follower::AnyAtOrBefore

**Purpose:** Same as AnyBefore but INCLUDES elements AT the boundary.

**Parameters:** `offset_type delay` - Offset from driver's range

**Boundary:** `range.upper_stamp - delay` (INCLUSIVE)

**Behavior:** Captures stamps <= boundary (not just <)

### Example 8.1: Inclusive Boundary

```
SETUP:
    delay = 5
    Driver range = {100, 100}
    boundary = 95
    
    Follower queue: [80, 85, 90, 95, 98, 105]
                                 ↑ AT boundary

CAPTURE with AnyBefore (exclusive):
    └─ Collects stamps < 95: [80, 85, 90]
    Output: [80, 85, 90]  ← stamp 95 NOT included

CAPTURE with AnyAtOrBefore (inclusive):
    └─ Collects stamps <= 95: [80, 85, 90, 95]
    Output: [80, 85, 90, 95]  ← stamp 95 INCLUDED
```

---

## 9. Follower Comparison Summary

| Follower | Captures | Requires Proof? | Can be Empty? | Use Case |
|----------|----------|-----------------|---------------|----------|
| `Before` | All < boundary | Yes (element after) | Yes | Historical context |
| `ClosestBefore` | 1 in window | Yes | No | Nearest-neighbor |
| `CountBefore` | Exactly N | Yes | No | Fixed history |
| `Ranged` | Range + boundaries | Yes (both ends) | No* | Interpolation |
| `MatchedStamp` | Exact match | N/A | No | Synchronized streams |
| `Latched` | 1, then holds | For first | No (after first) | Slow-updating state |
| `AnyBefore` | All < boundary | No (ALWAYS PRIMED) | Yes | Optional streams |
| `AnyAtOrBefore` | All <= boundary | No (ALWAYS PRIMED) | Yes | Optional + boundary |

*Ranged can capture no "in-range" elements if range is a point

### State Outcome Comparison

```
Given: Driver range = {100, 100}

follower::Before{5}:        boundary = 95 (exclusive)
follower::ClosestBefore{10, 5}: window = [85, 95)
follower::CountBefore{3, 5}: need 3 elements < 95
follower::MatchedStamp:     need stamp == 100
follower::Latched{50}:      boundary = 50

Queue A: [80, 90, 102]
├─ Before:       PRIMED (captures 80, 90)
├─ ClosestBefore: PRIMED (captures 90)
├─ CountBefore:  RETRY (only 2 elements < 95)
├─ MatchedStamp: ABORT (jumped past 100)
└─ Latched:      PRIMED (captures 80)

Queue B: [80, 90]
├─ Before:       RETRY (no proof, need element >= 95)
├─ ClosestBefore: RETRY (no proof)
├─ CountBefore:  RETRY (no proof)
├─ MatchedStamp: RETRY (stamp 100 might arrive)
└─ Latched:      PRIMED (captures 80)

Queue C: []
├─ Before:       RETRY (empty)
├─ ClosestBefore: RETRY (empty)
├─ CountBefore:  RETRY (empty)
├─ MatchedStamp: RETRY (empty)
├─ Latched:      RETRY (no latched value)
├─ AnyBefore:    PRIMED (empty is OK!) ← Only optional follower
└─ AnyAtOrBefore: PRIMED (empty is OK!)
```
