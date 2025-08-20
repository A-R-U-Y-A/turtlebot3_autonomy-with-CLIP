#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
import json
import os
import sqlite3
from datetime import datetime

class SemanticLocationManager(Node):
    def __init__(self):
        super().__init__('semantic_location_manager')
        
        # Parameters
        self.declare_parameter('database_file', '~/office_semantic_map.db')
        self.declare_parameter('coordinate_file', '~/office_exploration_data.json')
        self.declare_parameter('enable_numbered_queries', True)
        
        self.db_file = os.path.expanduser(
            self.get_parameter('database_file').get_parameter_value().string_value)
        self.coord_file = os.path.expanduser(
            self.get_parameter('coordinate_file').get_parameter_value().string_value)
        self.enable_numbered = self.get_parameter('enable_numbered_queries').value
        
        # Initialize database and load coordinates
        self.setup_database()
        self.load_saved_coordinates()
        
        # Subscribers
        self.query_sub = self.create_subscription(
            String, '/semantic_query', self.handle_query, 10)
        
        # Publishers
        self.location_pub = self.create_publisher(
            PoseStamped, '/semantic_location', 10)
        self.response_pub = self.create_publisher(
            String, '/semantic_response', 10)
        
        # Services
        self.refresh_service = self.create_service(
            SetBool, '/refresh_semantic_locations', self.refresh_locations)
        
        # Room database
        self.room_locations = {}
        self.numbered_rooms = {}
        
        self.get_logger().info("🏢 Semantic Location Manager started")
        self.publish_available_locations()
    
    def setup_database(self):
        """Initialize SQLite database for semantic locations"""
        try:
            self.conn = sqlite3.connect(self.db_file, check_same_thread=False)
            self.conn.execute('''
                CREATE TABLE IF NOT EXISTS semantic_locations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    room_type TEXT NOT NULL,
                    room_number INTEGER,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    confidence REAL DEFAULT 1.0,
                    discovered_time TEXT,
                    last_updated TEXT
                )
            ''')
            self.conn.commit()
            self.get_logger().info("✅ Database initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Database setup failed: {e}")
            self.conn = None
    
    def load_saved_coordinates(self):
        """Load saved exploration coordinates"""
        try:
            # Load main exploration data
            if os.path.exists(self.coord_file):
                with open(self.coord_file, 'r') as f:
                    data = json.load(f)
                
                # Load discovered rooms
                if 'discovered_rooms' in data:
                    for room_type, info in data['discovered_rooms'].items():
                        self.room_locations[room_type] = {
                            'x': info['coordinates']['x'],
                            'y': info['coordinates']['y'],
                            'confidence': info.get('confidence', 1.0),
                            'room_number': info.get('number', 0)
                        }
                        
                        # Add to database
                        self.store_location_in_db(room_type, 
                                                info['coordinates']['x'],
                                                info['coordinates']['y'],
                                                info.get('confidence', 1.0),
                                                info.get('number', 0))
                
                # Load numbered room access
                if 'room_coordinates' in data:
                    for num_str, room_info in data['room_coordinates'].items():
                        room_num = int(num_str)
                        self.numbered_rooms[room_num] = {
                            'room_type': room_info['room_type'],
                            'x': room_info['x'],
                            'y': room_info['y'],
                            'confidence': room_info.get('confidence', 1.0)
                        }
                
                self.get_logger().info(f"📍 Loaded {len(self.room_locations)} room locations")
            
            # Also try loading simplified coordinate file
            coord_simple_file = self.coord_file.replace('.json', '_coordinates.json')
            if os.path.exists(coord_simple_file):
                with open(coord_simple_file, 'r') as f:
                    numbered_data = json.load(f)
                
                for num_str, room_info in numbered_data.items():
                    room_num = int(num_str)
                    if room_num not in self.numbered_rooms:
                        self.numbered_rooms[room_num] = room_info
                        
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load coordinates: {e}")
    
    def store_location_in_db(self, room_type, x, y, confidence=1.0, room_number=0):
        """Store location in database"""
        if not self.conn:
            return
        
        try:
            # Check if location already exists
            cursor = self.conn.execute(
                'SELECT id FROM semantic_locations WHERE room_type = ? AND room_number = ?',
                (room_type, room_number))
            
            if cursor.fetchone():
                # Update existing
                self.conn.execute('''
                    UPDATE semantic_locations 
                    SET x = ?, y = ?, confidence = ?, last_updated = ?
                    WHERE room_type = ? AND room_number = ?
                ''', (x, y, confidence, datetime.now().isoformat(), 
                     room_type, room_number))
            else:
                # Insert new
                self.conn.execute('''
                    INSERT INTO semantic_locations 
                    (room_type, room_number, x, y, confidence, discovered_time, last_updated)
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                ''', (room_type, room_number, x, y, confidence, 
                     datetime.now().isoformat(), datetime.now().isoformat()))
            
            self.conn.commit()
        except Exception as e:
            self.get_logger().error(f"❌ Database store failed: {e}")
    
    def handle_query(self, msg):
        """Handle semantic location queries"""
        query = msg.data.lower().strip()
        self.get_logger().info(f"🔍 Processing query: '{query}'")
        
        location = None
        response_text = ""
        
        # Handle numbered queries (1, 2, 3, etc.)
        if self.enable_numbered and query.isdigit():
            room_num = int(query)
            if room_num in self.numbered_rooms:
                room_info = self.numbered_rooms[room_num]
                location = (room_info['x'], room_info['y'])
                response_text = f"🎯 Room #{room_num}: {room_info['room_type']} at ({room_info['x']:.2f}, {room_info['y']:.2f})"
            else:
                response_text = f"❌ Room #{room_num} not found. Available rooms: {list(self.numbered_rooms.keys())}"
        
        # Handle room type queries
        elif query in self.room_locations:
            room_info = self.room_locations[query]
            location = (room_info['x'], room_info['y'])
            response_text = f"🎯 Found {query} at ({room_info['x']:.2f}, {room_info['y']:.2f})"
        
        # Handle partial matches
        else:
            matches = []
            for room_type in self.room_locations.keys():
                if query in room_type.lower() or room_type.lower() in query:
                    matches.append(room_type)
            
            if len(matches) == 1:
                room_type = matches[0]
                room_info = self.room_locations[room_type]
                location = (room_info['x'], room_info['y'])
                response_text = f"🎯 Found {room_type} at ({room_info['x']:.2f}, {room_info['y']:.2f})"
            elif len(matches) > 1:
                response_text = f"🤔 Multiple matches found: {', '.join(matches)}. Please be more specific."
            else:
                response_text = f"❌ Location '{query}' not found. Available: {list(self.room_locations.keys())}"
        
        # Publish response
        self.publish_response(response_text)
        
        # Publish location if found
        if location:
            self.publish_location(location[0], location[1])
    
    def publish_location(self, x, y):
        """Publish semantic location as PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # No rotation
        
        self.location_pub.publish(pose)
        self.get_logger().info(f"📍 Published location: ({x:.2f}, {y:.2f})")
    
    def publish_response(self, text):
        """Publish text response"""
        response = String()
        response.data = text
        self.response_pub.publish(response)
        self.get_logger().info(f"💬 Response: {text}")
    
    def publish_available_locations(self):
        """Publish available locations on startup"""
        if self.room_locations:
            locations_list = list(self.room_locations.keys())
            if self.enable_numbered:
                numbered_list = [f"{num}: {info['room_type']}" 
                               for num, info in self.numbered_rooms.items()]
                available_text = f"🏢 Available locations: {', '.join(locations_list)}\n📍 Numbered access: {', '.join(numbered_list)}"
            else:
                available_text = f"🏢 Available locations: {', '.join(locations_list)}"
        else:
            available_text = "❌ No semantic locations loaded. Run exploration first."
        
        self.publish_response(available_text)
    
    def refresh_locations(self, request, response):
        """Service to refresh location data"""
        try:
            self.room_locations.clear()
            self.numbered_rooms.clear()
            self.load_saved_coordinates()
            self.publish_available_locations()
            
            response.success = True
            response.message = f"✅ Refreshed {len(self.room_locations)} locations"
            self.get_logger().info("🔄 Location data refreshed")
            
        except Exception as e:
            response.success = False
            response.message = f"❌ Refresh failed: {str(e)}"
            self.get_logger().error(f"❌ Refresh failed: {e}")
        
        return response
    
    def add_new_location(self, room_type, x, y, confidence=1.0):
        """Add new semantic location (called by other nodes)"""
        # Assign room number
        room_number = len(self.room_locations) + 1
        
        # Store in memory
        self.room_locations[room_type] = {
            'x': x, 'y': y, 
            'confidence': confidence,
            'room_number': room_number
        }
        
        if self.enable_numbered:
            self.numbered_rooms[room_number] = {
                'room_type': room_type,
                'x': x, 'y': y,
                'confidence': confidence
            }
        
        # Store in database
        self.store_location_in_db(room_type, x, y, confidence, room_number)
        
        self.get_logger().info(f"➕ Added new location: {room_type} #{room_number} at ({x:.2f}, {y:.2f})")
        
        # Update available locations
        self.publish_available_locations()

def main(args=None):
    rclpy.init(args=args)
    manager = SemanticLocationManager()
    
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.get_logger().info("🔄 Semantic Location Manager shutting down...")
    finally:
        if manager.conn:
            manager.conn.close()
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()