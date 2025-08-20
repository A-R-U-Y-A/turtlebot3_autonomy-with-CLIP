#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
import json
import os
from datetime import datetime
import sqlite3

class ExplorationDataLogger(Node):
    def __init__(self):
        super().__init__('exploration_data_logger')
        
        # Parameters
        self.declare_parameter('output_file', '~/office_exploration_data.json')
        self.declare_parameter('auto_save_interval', 30.0)
        
        self.output_file = os.path.expanduser(
            self.get_parameter('output_file').get_parameter_value().string_value)
        self.auto_save_interval = self.get_parameter('auto_save_interval').get_parameter_value().double_value
        
        # Subscribers
        self.semantic_sub = self.create_subscription(
            String, '/semantic_labels', self.semantic_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)
        
        # Data storage
        self.exploration_data = {
            'session_info': {
                'start_time': datetime.now().isoformat(),
                'map_file': None,
                'total_rooms_discovered': 0
            },
            'discovered_rooms': {},
            'exploration_goals': [],
            'planned_paths': [],
            'room_coordinates': {}
        }
        
        # Room counter for numbered access
        self.room_counter = 1
        
        # Auto-save timer
        self.timer = self.create_timer(self.auto_save_interval, self.auto_save_callback)
        
        # Database connection
        self.setup_database()
        
        self.get_logger().info(f"📊 Data logger started - saving to {self.output_file}")
    
    def setup_database(self):
        """Connect to semantic database"""
        try:
            db_path = os.path.expanduser('~/office_semantic_map.db')
            self.conn = sqlite3.connect(db_path, check_same_thread=False)
        except Exception as e:
            self.get_logger().error(f"Database connection failed: {e}")
            self.conn = None
    
    def semantic_callback(self, msg):
        """Process semantic label detections"""
        try:
            # Parse semantic message: "🏢 room_type at (x, y) [conf: confidence]"
            content = msg.data
            if 'at (' in content and ')' in content:
                # Extract room type
                room_type = content.split(' at (')[0].replace('🏢 ', '').strip()
                
                # Extract coordinates
                coord_part = content.split('at (')[1].split(')')[0]
                x, y = map(float, coord_part.split(', '))
                
                # Extract confidence
                conf_part = content.split('[conf: ')[1].split(']')[0]
                confidence = float(conf_part)
                
                # Check if this is a new high-confidence detection
                room_id = f"{room_type}_{self.room_counter}"
                
                if room_type not in self.exploration_data['discovered_rooms'] or \
                   confidence > self.exploration_data['discovered_rooms'][room_type].get('confidence', 0):
                    
                    self.exploration_data['discovered_rooms'][room_type] = {
                        'room_id': room_id,
                        'number': self.room_counter,
                        'coordinates': {'x': x, 'y': y},
                        'confidence': confidence,
                        'discovered_time': datetime.now().isoformat()
                    }
                    
                    # Add to room coordinates for easy access
                    self.exploration_data['room_coordinates'][str(self.room_counter)] = {
                        'room_type': room_type,
                        'x': x,
                        'y': y,
                        'confidence': confidence
                    }
                    
                    self.room_counter += 1
                    self.exploration_data['session_info']['total_rooms_discovered'] = len(
                        self.exploration_data['discovered_rooms'])
                    
                    self.get_logger().info(f"🏷️ Logged room #{self.room_counter-1}: {room_type} at ({x:.2f}, {y:.2f})")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing semantic data: {e}")
    
    def goal_callback(self, msg):
        """Log exploration goals"""
        goal_data = {
            'timestamp': datetime.now().isoformat(),
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'frame_id': msg.header.frame_id
        }
        self.exploration_data['exploration_goals'].append(goal_data)
    
    def path_callback(self, msg):
        """Log planned paths"""
        if len(msg.poses) == 0:
            return
            
        path_data = {
            'timestamp': datetime.now().isoformat(),
            'waypoints': []
        }
        
        for pose in msg.poses:
            waypoint = {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y
            }
            path_data['waypoints'].append(waypoint)
        
        self.exploration_data['planned_paths'].append(path_data)
    
    def auto_save_callback(self):
        """Automatically save data"""
        self.save_data()
    
    def save_data(self):
        """Save exploration data to JSON file"""
        try:
            # Update session info
            self.exploration_data['session_info']['last_updated'] = datetime.now().isoformat()
            
            # Save to JSON file
            os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
            with open(self.output_file, 'w') as f:
                json.dump(self.exploration_data, f, indent=2)
            
            # Also save a simplified coordinate file for easy loading
            coord_file = self.output_file.replace('.json', '_coordinates.json')
            with open(coord_file, 'w') as f:
                json.dump(self.exploration_data['room_coordinates'], f, indent=2)
            
            self.get_logger().info(f"💾 Saved exploration data: {len(self.exploration_data['discovered_rooms'])} rooms")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save data: {e}")
    
    def export_final_data(self):
        """Export final exploration summary"""
        try:
            # Create comprehensive summary
            summary = {
                'exploration_summary': {
                    'total_rooms_discovered': len(self.exploration_data['discovered_rooms']),
                    'room_types': list(self.exploration_data['discovered_rooms'].keys()),
                    'exploration_duration': 'calculated from start_time',
                    'total_goals': len(self.exploration_data['exploration_goals']),
                    'total_paths': len(self.exploration_data['planned_paths'])
                },
                'room_directory': self.exploration_data['discovered_rooms'],
                'numbered_access': self.exploration_data['room_coordinates']
            }
            
            summary_file = self.output_file.replace('.json', '_summary.json')
            with open(summary_file, 'w') as f:
                json.dump(summary, f, indent=2)
            
            self.get_logger().info(f"📋 Final summary exported to {summary_file}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to export summary: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationDataLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("💾 Saving final exploration data...")
        node.save_data()
        node.export_final_data()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()