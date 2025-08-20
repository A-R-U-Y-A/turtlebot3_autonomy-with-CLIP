#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import sqlite3
import os

class SemanticQueryService(Node):
    def __init__(self):
        super().__init__('semantic_query_service')
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.response_pub = self.create_publisher(String, '/semantic_response', 10)
        
        # Subscribers
        self.query_sub = self.create_subscription(
            String, '/semantic_query', self.handle_query, 10)
        
        # Database connection
        db_path = os.path.expanduser('~/semantic_map.db')
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        
        self.get_logger().info("Semantic query service initialized")
    
    def handle_query(self, msg):
        """Handle incoming semantic queries"""
        query = msg.data.lower().strip()
        self.get_logger().info(f"Processing query: '{query}'")
        
        # Parse query and search database
        location = self.find_semantic_location(query)
        
        if location:
            x, y, label, confidence = location
            
            # Publish navigation goal
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(x)
            goal.pose.position.y = float(y)
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal)
            
            # Send response
            response = String()
            response.data = f"Navigating to {label} at ({x:.2f}, {y:.2f})"
            self.response_pub.publish(response)
            
            self.get_logger().info(f"Sent navigation goal to {label}")
        else:
            response = String()
            response.data = f"Location '{query}' not found in semantic map"
            self.response_pub.publish(response)
    
    def find_semantic_location(self, query):
        """Find best matching semantic location"""
        cursor = self.conn.cursor()
        
        # Try exact match first
        cursor.execute('''
            SELECT x, y, semantic_label, confidence 
            FROM semantic_locations 
            WHERE LOWER(semantic_label) = ? 
            ORDER BY confidence DESC LIMIT 1
        ''', (query,))
        
        result = cursor.fetchone()
        if result:
            return result
        
        # Try partial match
        cursor.execute('''
            SELECT x, y, semantic_label, confidence 
            FROM semantic_locations 
            WHERE LOWER(semantic_label) LIKE ? 
            ORDER BY confidence DESC LIMIT 1
        ''', (f'%{query}%',))
        
        return cursor.fetchone()

def main(args=None):
    rclpy.init(args=args)
    node = SemanticQueryService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()