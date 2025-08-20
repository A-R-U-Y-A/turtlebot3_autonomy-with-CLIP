#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
import numpy as np
import random
import math
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class RRTNode:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        
        # Parameters
        self.declare_parameter('planning_timeout', 30.0)
        self.declare_parameter('step_size', 0.3)
        self.declare_parameter('goal_tolerance', 0.4)
        self.declare_parameter('max_iterations', 10000)
        
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.step_size = self.get_parameter('step_size').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_iterations = self.get_parameter('max_iterations').value
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action server for path planning
        self.action_server = ActionServer(
            self, ComputePathToPose, '/compute_path_to_pose', self.compute_path_callback)
        
        # Service client for costmap
        self.costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Current costmap
        self.costmap = None
        self.costmap_info = None
        
        self.get_logger().info("🗺️ RRT Path Planner initialized")
    
    async def compute_path_callback(self, goal_handle):
        """Compute path using RRT algorithm"""
        self.get_logger().info("🎯 Computing RRT path...")
        
        request = goal_handle.request
        start_pose = request.start
        goal_pose = request.goal
        
        # Get current costmap
        if not await self.get_costmap():
            goal_handle.abort()
            return ComputePathToPose.Result()
        
        # Convert poses to grid coordinates
        start_grid = self.world_to_grid(start_pose.pose.position.x, start_pose.pose.position.y)
        goal_grid = self.world_to_grid(goal_pose.pose.position.x, goal_pose.pose.position.y)
        
        if not start_grid or not goal_grid:
            self.get_logger().error("❌ Invalid start or goal position")
            goal_handle.abort()
            return ComputePathToPose.Result()
        
        # Run RRT algorithm
        path_nodes = self.rrt_search(start_grid, goal_grid)
        
        if not path_nodes:
            self.get_logger().error("❌ RRT failed to find path")
            goal_handle.abort()
            return ComputePathToPose.Result()
        
        # Convert path to ROS format
        path_msg = self.create_path_message(path_nodes, start_pose.header.frame_id)
        
        # Publish path
        self.path_pub.publish(path_msg)
        
        # Return result
        result = ComputePathToPose.Result()
        result.path = path_msg
        
        goal_handle.succeed()
        self.get_logger().info(f"✅ RRT path computed with {len(path_nodes)} waypoints")
        return result
    
    async def get_costmap(self):
        """Get current global costmap"""
        if not self.costmap_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Costmap service not available")
            return False
        
        request = GetCostmap.Request()
        try:
            response = await self.costmap_client.call_async(request)
            self.costmap = np.array(response.map.data).reshape(
                response.map.info.height, response.map.info.width)
            self.costmap_info = response.map.info
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Failed to get costmap: {e}")
            return False
    
    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid coordinates"""
        if not self.costmap_info:
            return None
        
        grid_x = int((world_x - self.costmap_info.origin.position.x) / self.costmap_info.resolution)
        grid_y = int((world_y - self.costmap_info.origin.position.y) / self.costmap_info.resolution)
        
        if (0 <= grid_x < self.costmap_info.width and 
            0 <= grid_y < self.costmap_info.height):
            return (grid_x, grid_y)
        return None
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        if not self.costmap_info:
            return None
        
        world_x = grid_x * self.costmap_info.resolution + self.costmap_info.origin.position.x
        world_y = grid_y * self.costmap_info.resolution + self.costmap_info.origin.position.y
        return (world_x, world_y)
    
    def is_collision_free(self, x, y):
        """Check if point is collision-free"""
        if (x < 0 or x >= self.costmap_info.width or 
            y < 0 or y >= self.costmap_info.height):
            return False
        
        # Check for obstacles (high cost values)
        return self.costmap[y, x] < 50  # Threshold for free space
    
    def is_line_collision_free(self, x1, y1, x2, y2):
        """Check if line segment is collision-free using Bresenham's algorithm"""
        points = self.bresenham_line(x1, y1, x2, y2)
        return all(self.is_collision_free(x, y) for x, y in points)
    
    def bresenham_line(self, x1, y1, x2, y2):
        """Generate points on line using Bresenham's algorithm"""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        x, y = x1, y1
        while True:
            points.append((x, y))
            if x == x2 and y == y2:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def distance(self, node1, node2):
        """Calculate Euclidean distance between nodes"""
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def get_random_point(self):
        """Generate random point in free space"""
        for _ in range(100):  # Max attempts
            x = random.randint(0, self.costmap_info.width - 1)
            y = random.randint(0, self.costmap_info.height - 1)
            if self.is_collision_free(x, y):
                return RRTNode(x, y)
        return None
    
    def get_nearest_node(self, tree, target_node):
        """Find nearest node in tree to target"""
        if not tree:
            return None
        
        min_dist = float('inf')
        nearest = None
        
        for node in tree:
            dist = self.distance(node, target_node)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        
        return nearest
    
    def steer(self, from_node, to_node):
        """Steer from one node toward another with step size limit"""
        dist = self.distance(from_node, to_node)
        
        if dist <= self.step_size / self.costmap_info.resolution:
            return to_node
        
        # Scale step
        scale = (self.step_size / self.costmap_info.resolution) / dist
        new_x = int(from_node.x + (to_node.x - from_node.x) * scale)
        new_y = int(from_node.y + (to_node.y - from_node.y) * scale)
        
        return RRTNode(new_x, new_y, from_node)
    
    def rrt_search(self, start, goal):
        """RRT path planning algorithm"""
        start_node = RRTNode(start[0], start[1])
        goal_node = RRTNode(goal[0], goal[1])
        
        tree = [start_node]
        
        for i in range(self.max_iterations):
            # Bias toward goal 10% of the time
            if random.random() < 0.1:
                random_node = goal_node
            else:
                random_node = self.get_random_point()
                if not random_node:
                    continue
            
            # Find nearest node
            nearest_node = self.get_nearest_node(tree, random_node)
            if not nearest_node:
                continue
            
            # Steer toward random point
            new_node = self.steer(nearest_node, random_node)
            
            # Check if path is collision-free
            if self.is_line_collision_free(nearest_node.x, nearest_node.y, 
                                         new_node.x, new_node.y):
                new_node.parent = nearest_node
                tree.append(new_node)
                
                # Check if we reached the goal
                if self.distance(new_node, goal_node) <= self.goal_tolerance / self.costmap_info.resolution:
                    goal_node.parent = new_node
                    return self.extract_path(goal_node)
                
                if i % 1000 == 0:
                    self.get_logger().info(f"🔄 RRT iteration {i}, tree size: {len(tree)}")
        
        self.get_logger().warn("⚠️ RRT max iterations reached without finding path")
        return None
    
    def extract_path(self, goal_node):
        """Extract path from goal to start"""
        path = []
        current = goal_node
        
        while current is not None:
            path.append(current)
            current = current.parent
        
        return path[::-1]  # Reverse to get start-to-goal path
    
    def create_path_message(self, path_nodes, frame_id):
        """Create ROS Path message from path nodes"""
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for node in path_nodes:
            pose = PoseStamped()
            pose.header = path_msg.header
            
            world_coords = self.grid_to_world(node.x, node.y)
            if world_coords:
                pose.pose.position.x = world_coords[0]
                pose.pose.position.y = world_coords[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0  # No rotation
                
                path_msg.poses.append(pose)
        
        return path_msg

def main(args=None):
    rclpy.init(args=args)
    planner = RRTPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info("🔄 RRT Planner shutting down...")
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()