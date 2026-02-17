# Flow Execution Examples

This directory contains detailed execution examples demonstrating how Flow's capture mechanisms work with various driver and follower combinations. These examples are designed to help developers understand:

- How data flows through the synchronization pipeline
- When and why captures succeed, retry, or fail
- Edge cases and potential pitfalls
- Message dropping scenarios and how to avoid them

## Document Index

| Document | Description |
|----------|-------------|
| [Driver Examples](./drivers.md) | Detailed examples for all driver types (Next, Batch, Chunk, Throttled) |
| [Follower Examples](./followers.md) | Detailed examples for all follower types |
| [Combined Workflows](./combined_workflows.md) | Real-world scenarios combining drivers and followers |
| [Edge Cases & Failures](./edge_cases_and_failures.md) | Common failure scenarios and how to handle them |
| [Message Dropping Scenarios](./message_dropping.md) | When and why messages get dropped |

## Quick Reference: State Outcomes

| State | Meaning | Action |
|-------|---------|--------|
| `PRIMED` | Capture successful | Process data |
| `RETRY` | Need more data | Wait and try again |
| `ABORT` | Cannot synchronize | Frame dropped, continue |
| `TIMEOUT` | Wait expired (MT only) | Retry or handle timeout |
| `ERROR_DRIVER_LOWER_BOUND_EXCEEDED` | Timestamp violation | Critical error |
| `SKIP_FRAME_QUEUE_PRECONDITION` | Queue monitor rejected | Continue to next frame |

## Visual Legend

In the execution traces, we use the following notation:

```
Queue State:  [oldest] ← ← ← [newest]
              [0, 1, 2, 3, 4, 5]
                ↑
              Elements ordered by timestamp

Capture Range: {lower_stamp, upper_stamp}
               {10, 15} means timestamps 10 to 15

Timeline:
    t=0   t=5   t=10   t=15   t=20
    |-----|-----|------|------|
    
    ▼ = captured element
    ✗ = removed element  
    ○ = retained element
```

## Common Patterns

### Pattern 1: Frame-by-Frame Processing
```cpp
driver::Next + follower::ClosestBefore
// Use when: Processing data one frame at a time with nearest-neighbor matching
```

### Pattern 2: Sliding Window Analysis
```cpp
driver::Batch + follower::Ranged
// Use when: Need overlapping windows for moving average, etc.
```

### Pattern 3: Non-Overlapping Batch Processing
```cpp
driver::Chunk + follower::Before
// Use when: Processing fixed-size batches without overlap
```

### Pattern 4: Rate-Limited Processing
```cpp
driver::Throttled + follower::Latched
// Use when: Input rate higher than processing rate, need stable state
```

### Pattern 5: Optional Data Streams
```cpp
driver::Next + follower::AnyBefore
// Use when: Some data streams may not always have data
```
