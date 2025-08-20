#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import math
from datetime import datetime

class NavigationStatusMonitor(Node):
    def __init__(self):
        super().__init__('navigation_status_monitor')
        
        # Parameters
        self.declare_parameter('status_publish_rate', 2.0)
        
        self.publish_rate = self.get_parameter('status_publish_rate').value
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/system_status', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.nav_status_sub = self.create_subscription(
            String, '/navigation_status', self.nav_status_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Status data
        self.robot_pose = None
        self.robot_velocity = None
        self.last_cmd_vel = None
        self.navigation_status = "Idle"
        self.battery_level = None
        self.battery_voltage = None
        self.obstacle_distance = None
        self.system_uptime = datetime.now()
        
        # Movement tracking
        self.total_distance = 0.0
        self.last_position = None
        self.is_moving = False
        self.stuck_counter = 0
        self.last_significant_movement = datetime.now()
        
        # Timers
        self.status_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_status_update)
        self.diagnostics_timer = self.create_timer(
            5.0, self.publish_diagnostics)
        
        self.get_logger().info("📊 Navigation Status Monitor started")
    
    def odom_callback(self, msg):
        """Process odometry data"""
        current_pose = msg.pose.pose
        self.robot_pose = current_pose
        self.robot_velocity = msg.twist.twist
        
        # Calculate movement distance
        if self.last_position:
            dx = current_pose.position.x - self.last_position.x
            dy = current_pose.position.y - self.last_position.y
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance += distance
            
            # Check if robot is moving significantly
            if distance > 0.01:  # 1cm threshold
                self.is_moving = True
                self.last_significant_movement = datetime.now()
                self.stuck_counter = 0
            else:
                if self.is_moving:
                    time_since_movement = (datetime.now() - self.last_significant_movement).total_seconds()
                    if time_since_movement > 5.0:  # 5 seconds without movement
                        self.is_moving = False
                        if self.last_cmd_vel and (abs(self.last_cmd_vel.linear.x) > 0.01 or abs(self.last_cmd_vel.angular.z) > 0.01):
                            self.stuck_counter += 1
        
        self.last_position = current_pose.position
    
    def cmd_vel_callback(self, msg):
        """Process velocity commands"""
        self.last_cmd_vel = msg
    
    def nav_status_callback(self, msg):
        """Process navigation status updates"""
        self.navigation_status = msg.data
    
    def battery_callback(self, msg):
        """Process battery status"""
        self.battery_level = msg.percentage * 100  # Convert to percentage
        self.battery_voltage = msg.voltage
    
    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        if msg.ranges:
            # Find minimum distance in front sector (±30 degrees)
            front_ranges = []
            total_rays = len(msg.ranges)
            front_sector = int(total_rays * 0.167)  # ±30 degrees ≈ 1/6 of 360°
            
            # Front sector indices
            start_idx = total_rays - front_sector
            end_idx = front_sector
            
            # Collect front ranges
            for i in range(start_idx, total_rays):
                if msg.range_min <= msg.ranges[i] <= msg.range_max:
                    front_ranges.append(msg.ranges[i])
            
            for i in range(0, end_idx):
                if msg.range_min <= msg.ranges[i] <= msg.range_max:
                    front_ranges.append(msg.ranges[i])
            
            if front_ranges:
                self.obstacle_distance = min(front_ranges)
    
    def publish_status_update(self):
        """Publish regular status updates"""
        status_lines = []
        
        # System uptime
        uptime = datetime.now() - self.system_uptime
        status_lines.append(f"🕐 Uptime: {str(uptime).split('.')[0]}")
        
        # Robot position
        if self.robot_pose:
            x = self.robot_pose.position.x
            y = self.robot_pose.position.y
            status_lines.append(f"📍 Position: ({x:.2f}, {y:.2f})")
            status_lines.append(f"🛣️ Total distance: {self.total_distance:.2f}m")
        
        # Movement status
        if self.robot_velocity:
            linear_speed = math.sqrt(
                self.robot_velocity.linear.x**2 + 
                self.robot_velocity.linear.y**2)
            angular_speed = abs(self.robot_velocity.angular.z)
            
            if self.is_moving:
                status_lines.append(f"🏃 Moving: {linear_speed:.2f}m/s, {math.degrees(angular_speed):.1f}°/s")
            else:
                status_lines.append("⏸️ Stationary")
        
        # Navigation status
        status_lines.append(f"🧭 Navigation: {self.navigation_status}")
        
        # Battery status
        if self.battery_level is not None:
            battery_icon = self.get_battery_icon(self.battery_level)
            status_lines.append(f"{battery_icon} Battery: {self.battery_level:.1f}%")
        
        # Obstacle detection
        if self.obstacle_distance is not None:
            if self.obstacle_distance < 0.3:
                status_lines.append(f"⚠️ Obstacle: {self.obstacle_distance:.2f}m ahead")
            elif self.obstacle_distance < 0.6:
                status_lines.append(f"🔶 Obstacle: {self.obstacle_distance:.2f}m ahead")
        
        # Stuck detection
        if self.stuck_counter > 0:
            status_lines.append(f"🚫 Possible stuck condition detected ({self.stuck_counter} times)")
        
        # Publish combined status
        status_msg = String()
        status_msg.data = " | ".join(status_lines)
        self.status_pub.publish(status_msg)
    
    def publish_diagnostics(self):
        """Publish detailed diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Navigation system status
        nav_diag = DiagnosticStatus()
        nav_diag.name = "Navigation System"
        nav_diag.hardware_id = "turtlebot3_navigation"
        
        if "progress" in self.navigation_status.lower():
            nav_diag.level = DiagnosticStatus.OK
            nav_diag.message = "Navigation active"
        elif "failed" in self.navigation_status.lower() or "error" in self.navigation_status.lower():
            nav_diag.level = DiagnosticStatus.ERROR
            nav_diag.message = "Navigation error"
        elif self.stuck_counter > 2:
            nav_diag.level = DiagnosticStatus.WARN
            nav_diag.message = "Robot may be stuck"
        else:
            nav_diag.level = DiagnosticStatus.OK
            nav_diag.message = "Navigation idle"
        
        nav_diag.values = [
            KeyValue(key="status", value=self.navigation_status),
            KeyValue(key="stuck_count", value=str(self.stuck_counter)),
            KeyValue(key="is_moving", value=str(self.is_moving))
        ]
        
        # Battery diagnostics
        battery_diag = DiagnosticStatus()
        battery_diag.name = "Battery System"
        battery_diag.hardware_id = "turtlebot3_battery"
        
        if self.battery_level is not None:
            if self.battery_level > 20:
                battery_diag.level = DiagnosticStatus.OK
                battery_diag.message = "Battery level good"
            elif self.battery_level > 10:
                battery_diag.level = DiagnosticStatus.WARN
                battery_diag.message = "Battery level low"
            else:
                battery_diag.level = DiagnosticStatus.ERROR
                battery_diag.message = "Battery level critical"
            
            battery_diag.values = [
                KeyValue(key="percentage", value=f"{self.battery_level:.1f}"),
                KeyValue(key="voltage", value=f"{self.battery_voltage:.2f}" if self.battery_voltage else "unknown")
            ]
        else:
            battery_diag.level = DiagnosticStatus.STALE
            battery_diag.message = "No battery data"
        
        # Obstacle detection diagnostics
        obstacle_diag = DiagnosticStatus()
        obstacle_diag.name = "Obstacle Detection"
        obstacle_diag.hardware_id = "turtlebot3_lidar"
        
        if self.obstacle_distance is not None:
            if self.obstacle_distance > 0.6:
                obstacle_diag.level = DiagnosticStatus.OK
                obstacle_diag.message = "Path clear"
            elif self.obstacle_distance > 0.3:
                obstacle_diag.level = DiagnosticStatus.WARN
                obstacle_diag.message = "Obstacle detected"
            else:
                obstacle_diag.level = DiagnosticStatus.ERROR
                obstacle_diag.message = "Obstacle very close"
            
            obstacle_diag.values = [
                KeyValue(key="min_distance", value=f"{self.obstacle_distance:.2f}")
            ]
        else:
            obstacle_diag.level = DiagnosticStatus.STALE
            obstacle_diag.message = "No sensor data"
        
        # Add all diagnostics
        diag_array.status = [nav_diag, battery_diag, obstacle_diag]
        self.diagnostics_pub.publish(diag_array)
    
    def get_battery_icon(self, level):
        """Get battery icon based on level"""
        if level > 75:
            return "🔋"
        elif level > 50:
            return "🔋"
        elif level > 25:
            return "🪫"
        else:
            return "🪫"
    
    def get_system_status_summary(self):
        """Get a summary of system status"""
        status = "OK"
        issues = []
        
        if self.battery_level is not None and self.battery_level < 20:
            issues.append("Low battery")
            status = "WARN"
        
        if self.obstacle_distance is not None and self.obstacle_distance < 0.3:
            issues.append("Obstacle close")
            status = "WARN"
        
        if self.stuck_counter > 2:
            issues.append("Robot stuck")
            status = "ERROR"
        
        if "failed" in self.navigation_status.lower():
            issues.append("Navigation failed")
            status = "ERROR"
        
        return status, issues

def main(args=None):
    rclpy.init(args=args)
    monitor = NavigationStatusMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("🔄 Navigation Status Monitor shutting down...")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()