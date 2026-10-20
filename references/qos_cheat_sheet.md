# ROS 2 QoS Compatibility Cheat Sheet

## Compatibility Matrix (Pub vs Sub)

A subscriber connects to a publisher if and only if:
- **Reliability**: Publisher Reliability >= Subscriber Reliability (`RELIABLE` publisher can satisfy `BEST_EFFORT` subscriber, but `BEST_EFFORT` publisher CANNOT satisfy `RELIABLE` subscriber).
- **Durability**: Publisher Durability >= Subscriber Durability (`TRANSIENT_LOCAL` publisher can satisfy `VOLATILE` subscriber, but `VOLATILE` publisher CANNOT satisfy `TRANSIENT_LOCAL` subscriber).

| Publisher Setting | Subscriber Setting | Compatible? | Resulting Behavior |
| :--- | :--- | :--- | :--- |
| `RELIABLE` | `RELIABLE` | **Yes** | Fully reliable delivery with ACKs. |
| `RELIABLE` | `BEST_EFFORT` | **Yes** | Subscriber drops packets if overwhelmed; pub retransmits for other subs. |
| `BEST_EFFORT` | `RELIABLE` | **NO** | **Silent failure: no messages delivered.** |
| `BEST_EFFORT` | `BEST_EFFORT` | **Yes** | Direct UDP-like low latency stream. |
| `TRANSIENT_LOCAL` | `TRANSIENT_LOCAL` | **Yes** | Late-joining subscriber gets last persisted message. |
| `TRANSIENT_LOCAL` | `VOLATILE` | **Yes** | Late-joining subscriber ignores past messages. |
| `VOLATILE` | `TRANSIENT_LOCAL` | **NO** | **Silent failure: no messages delivered.** |
| `VOLATILE` | `VOLATILE` | **Yes** | Only messages published while connected are received. |

---

## Standard Presets Quick Reference

### Sensor Streams (Cameras, PointClouds, IMU)
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

### Static Transforms & Maps (Latched Data)

```python
latched_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

### Control Commands & Mission Events

```python
control_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
```