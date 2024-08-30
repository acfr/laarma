# Data Logging
- [Continuous recording node](#continuous-recording)
- [Event-based recording node](#event-based-recording)
- [Documentation](#documentation)

### Continuous recording
```python
# continuous.launch.py
Node(
  package="data_logging",
  executable="basic_recorder",
  name="basic_recorder",
  parameters=[
    {
      "logging_dir": "/mnt/ssd/logging",
      "max_bag_size": 0,
      "max_bag_duration": 60,
      "name": "laarma_throttled",
      "storage_budget": 800,
      "sync": True,
      "config_file": "/mnt/ssd/config/basic.yaml"
    }
  ],
),
```
```yaml
# basic.yaml
topics:
  - [/topic/1]
  - [/topic/2]
  - [/topic/3]
video: /topic/image/4
```

### Event-based recording
```python
# event.launch.py
Node(
  package="data_logging",
  executable="event_recorder",
  name="event_recorder",
  parameters=[
    {
      "logging_dir": "/mnt/ssd/logging",
      "max_cache_size": 200000000,
      "name": "laarma_event",
      "snapshot_delay": 10,
      "snapshot_trigger_topic": "/event_trigger/event",
      "storage_budget": 2,
      "sync": True,
      "config_file": "/mnt/ssd/config/event.yaml"
    }
  ],
),
```
```yaml
# event.yaml
topics:
  - [/topic/5]
  - [/topic/6]
  - [/topic/7]
```

## Documentation
### Node parameters
| Parameter | Description | Example |
| ----------- | ----------- | --- |
| All node parameters
| `name` | Identifier used for bag names | `laarma_event` |
| `logging_dir` | An existing directory for logging data | `/mnt/ssd/logging` |
| `sync` | If enabled, recorder waits for a new minute | `True` |
| `config_file` | A file of topics to record and stream. | `/mnt/ssd/config/basic.yaml` | 
| `storage_budget` | Start deleting old data after storage_budget (in GB) is exceeded. | `1000` |
| Continuous node parameters
| `max_bag_size`| Split the bag after max_bag_size bytes. Use 0 to disable. | `0` |
| `max_bag_duration` | Split the bag after max_bag_duration seconds. Use 0 to disable. | `60` |
| Event-based node parameters
| `snapshot_trigger_topic` | Monitor this topic for events. | `/event_trigger/event` |
| `max_cache_size` | Maintain a circular buffer of max_cache_size bytes. | `200000000` |
| `snapshot_delay` | Wait snapshot_delay seconds after event trigger to dump buffer data to disk. | `10` |

### YAML parameters
```yaml
topics:
  - [/topic/8] # Record continously
  - [/topic/9, 09-17] # Record between 9AM-5PM
  - [/topic/10, 09-11, 13-19] # Record between 9AM-11AM and 1PM-7PM
  - [/topic/11, 0-1] # Record between midnight and 1AM
  - [/topic/12, 22-24] # Record between 10PM and midnight

# Only for continuous node
video: /topic/13/image # Record a low bandwidth H265 .mp4 video
```
