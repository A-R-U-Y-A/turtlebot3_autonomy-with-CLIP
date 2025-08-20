#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose, FollowPath
from nav2_msgs.srv import GetCostmap
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class SemanticNavigator(Node):
    def __init__(self):
        super().__init__('semantic_navigator')
        
        # Parameters
        self.declare_parameter('navigation_timeout', 60.0)
        self.declare_parameter('path_planning_service', '/compute_path_to_pose')
        self.declare_parameter('follow_path_action', '/follow_path')
        
        self.nav_timeout = self.get_parameter('navigation_timeout').value
        
        # TF2 setup for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.follow_path_client = ActionClient(self, FollowPath, '/follow_path')
        
        # Subscribers
        self.semantic_location_sub = self.create_subscription(
            PoseStamped, '/semantic_location', self.navigate_to_semantic_location, 10)
        self.nav_goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.navigate_to_pose, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.current_goal_pub = self.create_publisher(PoseStamped, '/current_navigation_goal', 10)
        
        # Navigation state
        self.current_goal = None
        self.navigation_active = False
        self.current_goal_handle = None
        
        self.get_logger().info("🧭 Semantic Navigator initialized")
        
        # Wait for action servers
        self.wait_for_action_servers()
    
    def wait_for_action_servers(self):
        """Wait for navigation action servers to be available"""
        self.get_logger().info("⏳ Waiting for navigation action servers...")
        
        if self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info("✅ Navigate to pose server ready")
        else:
            self.get_logger().error("❌ Navigate to pose server not available")
        
        if self.follow_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info("✅ Follow path server ready")
        else:
            self.get_logger().error("❌ Follow path server not available")
    
    def navigate_to_semantic_location(self, msg):
        """Navigate to a semantic location"""
        self.get_logger().info(f"🎯 Navigating to semantic location: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        self.navigate_to_pose(msg)
    
    def navigate_to_pose(self, goal_pose):
        """Navigate to a specific pose"""
        if self.navigation_active:
            self.get_logger().warn("⚠️ Navigation already in progress. Cancelling previous goal...")
            self.cancel_current_navigation()
        
        self.current_goal = goal_pose
        self.navigation_active = True
        
        # Publish current goal
        self.current_goal_pub.publish(goal_pose)
        
        # Create navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        
        # Send goal
        self.publish_status("🚀 Starting navigation...")
        
        send_goal_future = self.nav_client.send_goal_async(
            nav_goal, feedback_callback=self.navigation_feedback_callback)
        send_goal_future.add_done_callback(self.navigation_goal_response_callback)
    
    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Navigation goal rejected")
            self.publish_status("❌ Navigation goal rejected")
            self.navigation_active = False
            return
        
        self.get_logger().info("✅ Navigation goal accepted")
        self.current_goal_handle = goal_handle
        self.publish_status("🏃 Navigation in progress...")
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        
        if hasattr(feedback, 'distance_remaining'):
            distance = feedback.distance_remaining
            self.publish_status(f"🏃 Distance remaining: {distance:.2f}m")
        
        if hasattr(feedback, 'estimated_time_remaining'):
            time_remaining = feedback.estimated_time_remaining.sec
            self.publish_status(f"⏱️ ETA: {time_remaining}s")
    
    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎉 Navigation completed successfully!")
            self.publish_status("🎉 Navigation completed successfully!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("🛑 Navigation was cancelled")
            self.publish_status("🛑 Navigation cancelled")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("❌ Navigation failed/aborted")
            self.publish_status("❌ Navigation failed - obstacle or planning error")
        else:
            self.get_logger().warn(f"⚠️ Navigation ended with status: {status}")
            self.publish_status(f"⚠️ Navigation ended with status: {status}")
        
        self.navigation_active = False
        self.current_goal_handle = None
        self.current_goal = None
    
    def cancel_current_navigation(self):
        """Cancel current navigation"""
        if self.current_goal_handle and self.navigation_active:
            self.get_logger().info("🛑 Cancelling current navigation...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_callback)
    
    def cancel_response_callback(self, future):
        """Handle cancel response"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("✅ Navigation cancelled successfully")
        else:
            self.get_logger().warn("⚠️ Failed to cancel navigation")
        
        self.navigation_active = False
        self.current_goal_handle = None
    
    def publish_status(self, status_text):
        """Publish navigation status"""
        status_msg = String()
        status_msg.data = status_text
        self.status_pub.publish(status_msg)
        self.get_logger().info(f"📊 Status: {status_text}")
    
    def get_robot_pose(self):
        """Get current robot pose in map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to get robot pose: {e}")
            return None
    
    def emergency_stop(self):
        """Emergency stop - cancel all navigation"""
        self.get_logger().warn("🚨 EMERGENCY STOP!")
        self.cancel_current_navigation()
        self.publish_status("🚨 EMERGENCY STOP - All navigation cancelled")
    
    def navigate_to_coordinates(self, x, y, frame_id='map'):
        """Navigate to specific coordinates"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0  # No specific orientation
        
        self.navigate_to_pose(goal_pose)
    
    def is_navigation_active(self):
        """Check if navigation is currently active"""
        return self.navigation_active
    
    def get_current_goal(self):
        """Get current navigation goal"""
        return self.current_goal

def main(args=None):
    rclpy.init(args=args)
    navigator = SemanticNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("🔄 Semantic Navigator shutting down...")
        if navigator.navigation_active:
            navigator.emergency_stop()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()