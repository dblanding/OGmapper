# Incremental Occupancy Grid Mapper

A real-time SLAM mapping system that builds occupancy grid maps by listening to LIDAR scan and odometry pose data via MQTT. Supports incremental map building across multiple robot runs with pose interpolation for improved accuracy.

## Usage

* `uv run mapper.py` - Start fresh map
* `uv run mapper.py --load` - Continue existing map
* `uv run mapper.py --display` - View the current map
* `uv run mapper.py --display --file old_map.npy` - View specific map file

## Prerequisites

* Python 3.8+
* MQTT broker (e.g., Mosquitto) running on robot
* Robot publishing to MQTT topics:
  - `/robot/scan` - LIDAR scan data with timestamps
  - `/robot/pose` - Odometry pose (x, y, theta, timestamp)

## Key Features

### 1. Incremental Mapping
- Load and continue building existing maps across multiple robot runs
- Saves map state to `map.npy` and visualization to `map.png`
- Preserves log-odds values for proper evidence accumulation

### 2. Pose Interpolation
Instead of using the current pose when processing scans, the mapper:
- Stores the last 100 poses with timestamps in a history buffer
- Interpolates the exact robot pose at each scan point's timestamp
- Handles angle wraparound correctly at ±π
- Transforms each scan point using its interpolated pose for sub-scan accuracy

**Implementation:**
- `pose_history` - Ring buffer of timestamped poses
- `interpolate_angle()` - Handles circular angle interpolation
- `get_pose_at_time()` - Linear interpolation between bracketing poses
- `process_scan()` - Uses `get_pose_at_time(point['t'])` per point

### 3. Log-Odds Occupancy Grid
- **Occupied evidence**: +0.7 per hit
- **Free space evidence**: -0.4 per ray
- **Evidence ratio**: 1.75:1 (occupied favored)
- **Persistence**: ~2 free observations needed to cancel 1 occupied hit
- **Saturation**: Clamped to [-3.5, 3.5] to prevent overflow

**Conversion for visualization:**
- Log-odds → Probability: `P(occupied) = 1 / (1 + exp(-log_odds))`
- Thresholds: >65% = occupied (black), <35% = free (white), else unknown (gray)

### 4. Ray Tracing
- Uses Bresenham's line algorithm for efficient ray casting
- Marks cells along each ray as free space
- Marks endpoint as occupied obstacle

## Map Configuration

Default parameters (adjustable in `__init__`):
- **Resolution**: 0.05m (5cm per cell)
- **Size**: 400×400 cells (20m × 20m)
- **Origin**: (-3.0, -7.0) meters
- **Log-odds range**: [-3.5, 3.5]

## Output Files

- `map.npy` - NumPy array of log-odds grid values
- `map.png` - Visualization (black=occupied, white=free, gray=unknown)

## Example Workflow

1. **Start robot (including MQTT broker)**:

2. **First mapping run**:
   - Place robot in precise home position
      - Start scanner service
      - Start odometer service
      - Start Mapper `uv run mapper.py`
   - Drive robot around
   - Press Ctrl+C to save
3. **Continue mapping**:
   - Return robot to home position
      - Start scanner service
      - Start odometer service
      - Start Mapper `uv run mapper.py --load`
   - Drive robot along a new path (Explore new areas)
   - Press Ctrl+C to save
   - Map updates incrementally
4. **View results**:
   - At any time, inspect map `uv run mapper.py --display`

## Known Limitations

- **Pose drift**: Odometry-only localization accumulates error over long runs
- **No loop closure**: Revisiting areas doesn't correct drift
- **Static environment assumption**: Moving objects may create artifacts

