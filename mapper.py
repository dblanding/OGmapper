#!/usr/bin/env python3
"""
mapper.py - Build occupancy grid map from LIDAR and odometry
"""

import cv2
import paho.mqtt.client as mqtt
import json
import numpy as np
import time
import argparse
import sys
sys.path.insert(0, '../raspibot/robot')  # path to where topics.py is
from topics import Topics

class OccupancyGridMapper:
    def __init__(self, 
                 resolution=0.05,      # 5cm per cell
                 width=400,            # 20m wide (400 * 0.05)
                 height=400,           # 20m tall
                 origin_x=-3.0,       # Map origin (bottom-left corner)
                 origin_y=-7.0,
                 load_existing=False): # Load existing map
        
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        
        # Occupancy grid using log-odds
        # 0 = unknown, >0 = occupied, <0 = free
        if load_existing:
            self.load_map()
        else:
            self.grid = np.zeros((height, width), dtype=np.float32)
        
        # Current robot pose (fallback)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.pose_updated = False
        
        # Pose history for interpolation
        self.pose_history = []
        self.max_pose_history = 100
        
        # Latest scan
        self.latest_scan = None
        
        # Log-odds parameters
        self.l_occ = 0.7      # Log-odds for occupied
        self.l_free = -0.4    # Log-odds for free
        self.l_max = 3.5      # Clamp maximum
        self.l_min = -3.5     # Clamp minimum
        
        # MQTT
        self.client = mqtt.Client()
        self.client.username_pw_set("robot", "robot")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # Stats
        self.scans_processed = 0
        self.last_update_time = time.time()
        
    def on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            print("✓ Connected to MQTT broker")
            client.subscribe(Topics.LIDAR_SCAN)
            client.subscribe(Topics.ODOM_POSE)
            print(f"✓ Subscribed to {Topics.LIDAR_SCAN}")
            print(f"✓ Subscribed to {Topics.ODOM_POSE}")
        else:
            print(f"✗ Connection failed with code {rc}")
    
    def interpolate_angle(self, theta1, theta2, alpha):
        """Interpolate angle handling wraparound at ±π"""
        diff = theta2 - theta1
        # Normalize to [-pi, pi]
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return theta1 + alpha * diff
    
    def get_pose_at_time(self, timestamp):
        """Interpolate pose at a specific timestamp"""
        if not self.pose_history:
            return {'x': self.robot_x, 'y': self.robot_y, 'h': self.robot_theta, 't': timestamp}
        
        if len(self.pose_history) < 2:
            return self.pose_history[0]
        
        # If timestamp is before first pose, use first pose
        if timestamp <= self.pose_history[0]['t']:
            return self.pose_history[0]
        
        # If timestamp is after last pose, use last pose
        if timestamp >= self.pose_history[-1]['t']:
            return self.pose_history[-1]
        
        # Find bracketing poses
        for i in range(len(self.pose_history) - 1):
            t1 = self.pose_history[i]['t']
            t2 = self.pose_history[i + 1]['t']
            
            if t1 <= timestamp <= t2:
                # Linear interpolation
                if t2 == t1:
                    alpha = 0
                else:
                    alpha = (timestamp - t1) / (t2 - t1)
                
                p1 = self.pose_history[i]
                p2 = self.pose_history[i + 1]
                
                interpolated_pose = {
                    'x': p1['x'] + alpha * (p2['x'] - p1['x']),
                    'y': p1['y'] + alpha * (p2['y'] - p1['y']),
                    'h': self.interpolate_angle(p1['h'], p2['h'], alpha),
                    't': timestamp
                }
                
                return interpolated_pose
        
        # Fallback (shouldn't reach here)
        return self.pose_history[-1]
    
    def on_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            data = json.loads(msg.payload.decode())
            
            if msg.topic == Topics.ODOM_POSE:
                # Store pose with timestamp
                pose = {
                    'x': data['x'],
                    'y': data['y'],
                    'h': data['h'],
                    't': data['t']
                }
                
                # Update current pose (fallback)
                self.robot_x = pose['x']
                self.robot_y = pose['y']
                self.robot_theta = pose['h']
                self.pose_updated = True
                
                # Add to pose history
                self.pose_history.append(pose)
                
                # Maintain buffer size
                if len(self.pose_history) > self.max_pose_history:
                    self.pose_history.pop(0)
                
                #print(f"Robot pose: ({self.robot_x:.2f}, {self.robot_y:.2f}, {self.robot_theta:.2f}) [history: {len(self.pose_history)}]")
                
            elif msg.topic == Topics.LIDAR_SCAN:
                self.latest_scan = data
                
                # Process scan if we have pose
                if self.pose_updated:
                    self.process_scan(data)
                    
        except Exception as e:
            print(f"Error processing message: {e}")
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices"""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates"""
        x = grid_x * self.resolution + self.origin_x
        y = grid_y * self.resolution + self.origin_y
        return x, y
    
    def is_valid_cell(self, grid_x, grid_y):
        """Check if grid cell is within bounds"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing"""
        cells = []
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            cells.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            
            if e2 > -dy:
                err -= dy
                x += sx
            
            if e2 < dx:
                err += dx
                y += sy
        
        return cells

    def process_scan(self, scan_data):
        """Process incoming LIDAR scan data with timestamp-based pose interpolation"""
        try:
            if not scan_data or len(scan_data) == 0:
                return
            
            # Convert polar to cartesian and update map
            for point in scan_data:
                angle = point['a']
                distance = point['d']
                scan_timestamp = point['t']
                
                # Skip invalid readings
                if distance <= 0.1 or distance > 3:  # 3m max range
                    continue
                
                # Get interpolated pose at this scan point's timestamp
                pose = self.get_pose_at_time(scan_timestamp)
                
                robot_x = pose['x']
                robot_y = pose['y']
                robot_theta = pose['h']
                
                # Convert to cartesian (relative to robot)
                x_local = distance * np.cos(angle)
                y_local = distance * np.sin(angle)
                
                # Transform to global coordinates
                x_global = robot_x + x_local * np.cos(robot_theta) - y_local * np.sin(robot_theta)
                y_global = robot_y + x_local * np.sin(robot_theta) + y_local * np.cos(robot_theta)
                
                # Flip Y because scanner runs CW (apply to BOTH robot and obstacle)
                y_global_flipped = -y_global
                robot_y_flipped = -robot_y
                
                # Update map
                self.update_map(x_global, y_global_flipped, occupied=True)
                
                # Mark ray as free (using consistent flipped coordinate system)
                self.mark_ray_free(robot_x, robot_y_flipped, x_global, y_global_flipped)
            
            self.scans_processed += 1
            if self.scans_processed % 10 == 0:
                print(f"Processed {self.scans_processed} scans, pose history: {len(self.pose_history)} entries")
                
        except Exception as e:
            print(f"Error processing scan: {e}")

    def update_map(self, x, y, occupied=True):
        """Update a single cell in the occupancy grid using log-odds"""
        # Convert world coordinates to grid indices
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        
        # Check bounds
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            if occupied:
                self.grid[grid_y, grid_x] += self.l_occ  # +0.7
            else:
                self.grid[grid_y, grid_x] += self.l_free  # -0.4
            
            # Clamp to prevent overflow
            self.grid[grid_y, grid_x] = np.clip(self.grid[grid_y, grid_x], 
                                                self.l_min, self.l_max)

    def mark_ray_free(self, x0, y0, x1, y1):
        """Mark cells along a ray as free space using Bresenham's algorithm"""
        # Get grid coordinates
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)
        
        # Bresenham's line algorithm
        points = self.bresenham_line(gx0, gy0, gx1, gy1)
        
        for x, y in points:
            # Don't mark the endpoint (obstacle) as free
            if (x, y) == (gx1, gy1):
                continue
                
            if 0 <= x < self.width and 0 <= y < self.height:
                # Direct grid update
                self.grid[y, x] += self.l_free  # ← Changed from self.log_odds_free
                self.grid[y, x] = np.clip(self.grid[y, x], self.l_min, self.l_max)  # ← Use self.l_min, self.l_max


    def publish_map(self):
        """Publish current map state"""
        try:
            # Convert log-odds to probability (0-100)
            # P(occupied) = 1 / (1 + exp(-log_odds))
            prob_grid = 1.0 / (1.0 + np.exp(-self.grid))
            prob_grid = prob_grid * 100.0
            
            # Convert to integers (0-100, -1 for unknown)
            map_data = np.where(
                np.abs(self.grid) < 0.1,  # Unknown threshold
                -1,
                prob_grid.astype(np.int8)
            )
            
            map_msg = {
                'width': self.width,
                'height': self.height,
                'resolution': self.resolution,
                'origin_x': self.origin_x,
                'origin_y': self.origin_y,
                'data': map_data.flatten().tolist(),
                'timestamp': time.time()
            }
            
            self.client.publish(Topics.MAP_GRID, json.dumps(map_msg))
            
        except Exception as e:
            print(f"Error publishing map: {e}")

    def save_map(self, filename="map.npy"):
        """Save map to disk"""
        try:
            np.save(filename, self.grid)
            print(f"✓ Map saved to {filename}")
            
            # Convert log-odds to occupancy probability for display
            # P(occupied) = 1 / (1 + exp(-log_odds))
            prob = 1.0 / (1.0 + np.exp(-self.grid))
            prob_scaled = (prob * 100).astype(np.float32)
            
            # Check map stats
            occupied = prob_scaled > 65
            free = prob_scaled < 35
            unknown = ~(occupied | free)
            
            print(f"Map stats: {np.sum(occupied)} occupied, {np.sum(free)} free, {np.sum(unknown)} unknown")
            
            # Save as image
            map_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            map_img[unknown] = [128, 128, 128]  # Gray for unknown
            map_img[free] = [255, 255, 255]      # White for free
            map_img[occupied] = [0, 0, 0]        # Black for occupied
            
            cv2.imwrite('map.png', map_img)
            print(f"✓ Map image saved to map.png")
            
        except Exception as e:
            print(f"✗ Failed to save map: {e}")    

    def load_map(self, filename="map.npy"):
        """Load map from disk"""
        try:
            self.grid = np.load(filename)
            print(f"✓ Map loaded from {filename}")
            
            # Convert log-odds to occupancy probability for display
            # P(occupied) = 1 / (1 + exp(-log_odds))
            prob = 1.0 / (1.0 + np.exp(-self.grid))
            prob_scaled = (prob * 100).astype(np.float32)
            
            # Check map stats
            occupied = prob_scaled > 65
            free = prob_scaled < 35
            unknown = ~(occupied | free)
            print(f"Map stats: {np.sum(occupied)} occupied, {np.sum(free)} free, {np.sum(unknown)} unknown")
            
        except Exception as e:
            print(f"✗ Failed to load map: {e}")
            # Initialize empty grid if load fails
            self.grid = np.zeros((self.height, self.width), dtype=np.float32)
    
    def run(self):
        """Main loop"""
        print("Starting mapper...")
        print(f"Map size: {self.width}x{self.height} cells ({self.width*self.resolution}x{self.height*self.resolution} meters)")
        print(f"Resolution: {self.resolution} m/cell")
        print(f"Origin: ({self.origin_x}, {self.origin_y})")
        print(f"Log-odds: occupied={self.l_occ}, free={self.l_free}, range=[{self.l_min}, {self.l_max}]")
        
        try:
            self.client.connect("raspibot.local", 1883, 60)
            self.client.loop_start()
            
            # Keep running
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\nShutting down mapper...")
            self.save_map()
            self.client.loop_stop()
            self.client.disconnect()
            print("✓ Mapper stopped")


def display_map(filename="map.npy"):
    """Load and display map in a window (static method, no MQTT needed)"""
    try:
        # Load the map
        grid = np.load(filename)
        print(f"✓ Map loaded from {filename}")
        print(f"Grid shape: {grid.shape}")
        print(f"Grid dtype: {grid.dtype}")
        print(f"Value range: [{grid.min():.2f}, {grid.max():.2f}]")
        
        # Convert log-odds to occupancy probability
        # P(occupied) = 1 / (1 + exp(-log_odds))
        prob = 1.0 / (1.0 + np.exp(-grid))
        prob_scaled = (prob * 100).astype(np.float32)
        
        # Check map stats
        occupied = prob_scaled > 65
        free = prob_scaled < 35
        unknown = ~(occupied | free)
        print(f"Map stats: {np.sum(occupied)} occupied, {np.sum(free)} free, {np.sum(unknown)} unknown")
        
        # Create image
        height, width = grid.shape
        map_img = np.zeros((height, width, 3), dtype=np.uint8)
        map_img[unknown] = [128, 128, 128]  # Gray for unknown
        map_img[free] = [255, 255, 255]      # White for free
        map_img[occupied] = [0, 0, 0]        # Black for occupied
        
        # Scale up for better visibility (4x larger)
        map_img_scaled = cv2.resize(map_img, (width*4, height*4), interpolation=cv2.INTER_NEAREST)
        
        # Display in window
        cv2.imshow('Occupancy Grid Map', map_img_scaled)
        print("Displaying map... Press any key to close window")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("✓ Window closed")
        
    except Exception as e:
        print(f"✗ Failed to display map: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Occupancy Grid Mapper')
    parser.add_argument('--load', action='store_true', help='Load existing map.npy and continue mapping')
    parser.add_argument('--display', action='store_true', help='Display map.npy and exit')
    parser.add_argument('--file', type=str, default='map.npy', help='Map file to load/display (default: map.npy)')
    
    args = parser.parse_args()
    
    # If display mode, just show the map and exit (no MQTT, no mapper instance needed)
    if args.display:
        display_map(args.file)
    else:
        # Normal mapping mode
        mapper = OccupancyGridMapper(load_existing=args.load)
        mapper.run()
