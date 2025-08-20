#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
import json
import os
from datetime import datetime


class ExplorationMonitor(Node):
    """
    Monitors exploration progress and handles completion detection
    """
    
    def __init__(self):
        super().__init__('exploration_monitor')
        
        # Parameters
        self.declare_parameter('completion_check_interval', 10.0)
        self.declare_parameter('auto_save_on_completion', True)
        # use_sim_time is handled by the launch system - don't declare it here
        
        self.check_interval = self.get_parameter('completion_check_interval').value
        self.auto_save = self.get_parameter('auto_save_on_completion').value
        
        # State tracking
        self.exploration_complete = False
        self.last_pose = None
        self.pose_unchanged_count = 0
        self.max_unchanged_threshold = 10  # Consider complete if robot hasn't moved for 10 checks (more time)
        self.last_goal_time = None
        self.goal_timeout = 30.0  # 30 seconds without new goals
        
        # Publishers
        self.completion_pub = self.create_publisher(
            Bool, '/exploration/complete', 10
        )
        self.status_pub = self.create_publisher(
            String, '/exploration/status', 10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.pose_callback, 10
        )
        
        # Timer for periodic checks
        self.timer = self.create_timer(
            self.check_interval, self.check_completion
        )
        
        self.get_logger().info("🔍 Exploration monitor started")
    
    def pose_callback(self, msg):
        """Track robot goal changes (not pose changes)"""
        # This tracks when new goals are published, indicating active exploration
        self.last_goal_time = self.get_clock().now()
        
        current_pose = (msg.pose.position.x, msg.pose.position.y)
        
        if self.last_pose is None:
            self.last_pose = current_pose
            self.pose_unchanged_count = 0
        else:
            # Check if goal has changed significantly (within 0.5m for goals)
            distance = ((current_pose[0] - self.last_pose[0])**2 + 
                       (current_pose[1] - self.last_pose[1])**2)**0.5
            
            if distance < 0.5:
                self.pose_unchanged_count += 1
            else:
                self.pose_unchanged_count = 0
                self.last_pose = current_pose
    
    def check_completion(self):
        """Check if exploration should be considered complete"""
        if self.exploration_complete:
            return
        
        # Check multiple completion criteria
        goal_timeout_exceeded = False
        if self.last_goal_time is not None:
            time_since_goal = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
            goal_timeout_exceeded = time_since_goal > self.goal_timeout
        
        # Completion criteria: no new goals for timeout period AND robot hasn't moved
        if goal_timeout_exceeded and self.pose_unchanged_count >= self.max_unchanged_threshold:
            self.get_logger().info("🎯 Exploration appears complete - no new goals and robot stationary")
            self.mark_complete()
        
        # Publish current status
        status_msg = String()
        time_since_goal = 0
        if self.last_goal_time is not None:
            time_since_goal = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
        status_msg.data = f"Monitoring: unchanged_count={self.pose_unchanged_count}, time_since_goal={time_since_goal:.1f}s"
        self.status_pub.publish(status_msg)
    
    def mark_complete(self):
        """Mark exploration as complete and trigger save if enabled"""
        if self.exploration_complete:
            return
            
        self.exploration_complete = True
        
        # Publish completion signal
        completion_msg = Bool()
        completion_msg.data = True
        self.completion_pub.publish(completion_msg)
        
        self.get_logger().info("✅ Exploration marked as complete!")
        
        if self.auto_save:
            self.trigger_data_save()
    
    def trigger_data_save(self):
        """Trigger final data save"""
        try:
            # Create completion summary
            summary = {
                'completion_time': datetime.now().isoformat(),
                'status': 'completed',
                'final_pose': self.last_pose if self.last_pose else None,
                'completion_reason': 'robot_stationary'
            }
            
            # Save completion summary
            home_dir = os.path.expanduser('~')
            completion_file = os.path.join(home_dir, 'exploration_completion.json')
            
            with open(completion_file, 'w') as f:
                json.dump(summary, f, indent=2)
            
            self.get_logger().info(f"💾 Completion summary saved to {completion_file}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save completion data: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = ExplorationMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in exploration monitor: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()