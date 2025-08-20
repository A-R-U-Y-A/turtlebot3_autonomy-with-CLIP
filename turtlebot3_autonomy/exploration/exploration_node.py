#!/usr/bin/env python3
"""
Random Sampling Exploration Node for the robot to explore the environment autonomously while mapping the world
SUBSCRIBERS:
 sub_map (nav_msgs/OccupancyGrid) - represents a 2-D grid map, in which each cell represents the probability of occupancy.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from random import randrange
import time


class ExplorationNode(Node):

    def __init__(self):
        """Initialize exploration environment"""
        super().__init__('exploration_node')
        
        # Initialize rate
        self.rate = self.create_rate(1)  # 1 Hz

        # Action Client for Navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available!")
        else:
            self.get_logger().info("Navigation action server is ready")

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.completion = 0
        self.map_data = None
        self.exploration_complete = False
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscribers
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Wait for map data before starting
        self.get_logger().info("Waiting for map data...")
        time.sleep(3)

    def map_callback(self, data):
        """
        Callback function for map subscriber.
        Subscribes to /map to get the OccupancyGrid of the map.
        """
        if self.exploration_complete:
            return
            
        self.map_data = data
        valid = False
        max_attempts = 100  # Prevent infinite loops
        attempts = 0

        while not valid and attempts < max_attempts:
            attempts += 1
            
            # Get random point from map
            map_size = randrange(len(data.data))
            cell_value = data.data[map_size]

            # Check if this is a valid exploration point
            if self.is_valid_exploration_point(data, map_size, cell_value):
                valid = True
                
        if not valid:
            self.get_logger().warn("Could not find valid exploration point after 100 attempts")
            return

        # Convert grid coordinates to world coordinates
        self.x, self.y = self.grid_to_world(data, map_size)
        
        # Limit goal generation rate
        if self.completion % 2 == 0:
            self.completion += 1
            self.set_goal()

    def is_valid_exploration_point(self, data, map_size, cell_value):
        """
        Check if a point is valid for exploration:
        - Must be free space (value <= 0.2)
        - Must not be unknown (-1)
        - Must have frontier neighbors (adjacent to unknown areas)
        """
        # Must be free space and not unknown
        if cell_value == -1 or cell_value > 0.2:
            return False
            
        # Check neighbors for frontier detection
        return self.check_neighbors(data, map_size)

    def grid_to_world(self, data, map_size):
        """Convert grid index to world coordinates"""
        # Calculate row and column from linear index
        width = data.info.width
        row = map_size // width
        col = map_size % width

        # Convert to world coordinates using map info
        x = col * data.info.resolution + data.info.origin.position.x
        y = row * data.info.resolution + data.info.origin.position.y
        
        return x, y

    def set_goal(self):
        """Set goal position for navigation"""
        self.get_logger().info(f"Setting goal at ({self.x:.2f}, {self.y:.2f})")

        # Create and publish goal for data logging
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.x
        goal_pose.pose.position.y = self.y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_pose)

        # Create navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        # Send goal to navigation
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response from navigation server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by navigation server')
            self.completion += 1
            return

        self.get_logger().info('Goal accepted by navigation server')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle goal result from navigation server"""
        result = future.result().result
        status = future.result().status
        
        self.completion += 1

        # Goal reached
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded")

        # Goal aborted
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal aborted")

        # Goal canceled
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal canceled")
            
        else:
            self.get_logger().info(f"Goal finished with status: {status}")

    def check_neighbors(self, data, map_size):
        """
        Checks neighbors for random points on the map.
        Returns True if point has unknown neighbors and few obstacles.
        """
        unknowns = 0
        obstacles = 0
        width = data.info.width

        # Check 7x7 neighborhood around the point
        for x in range(-3, 4):
            for y in range(-3, 4):
                neighbor_index = map_size + (x * width) + y
                
                # Check bounds
                if neighbor_index < 0 or neighbor_index >= len(data.data):
                    continue
                    
                try:
                    neighbor_value = data.data[neighbor_index]
                    if neighbor_value == -1:  # Unknown
                        unknowns += 1
                    elif neighbor_value > 0.65:  # Obstacle
                        obstacles += 1
                except IndexError:
                    continue
                    
        # Valid frontier point: has unknown neighbors and few obstacles
        return unknowns > 0 and obstacles < 2


def main():
    """The main() function"""
    rclpy.init()
    
    try:
        explorer = ExplorationNode()
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Exploration node error: {e}")