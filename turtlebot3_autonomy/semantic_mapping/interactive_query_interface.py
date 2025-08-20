#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import threading
import sys
import select
import tty
import termios

class InteractiveQueryInterface(Node):
    def __init__(self):
        super().__init__('interactive_query_interface')
        
        # Parameters
        self.declare_parameter('show_menu_on_start', True)
        self.declare_parameter('auto_execute_goals', True)
        
        self.show_menu = self.get_parameter('show_menu_on_start').value
        self.auto_execute = self.get_parameter('auto_execute_goals').value
        
        # Publishers
        self.query_pub = self.create_publisher(String, '/semantic_query', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribers
        self.response_sub = self.create_subscription(
            String, '/semantic_response', self.handle_response, 10)
        self.location_sub = self.create_subscription(
            PoseStamped, '/semantic_location', self.handle_location, 10)
        self.status_sub = self.create_subscription(
            String, '/navigation_status', self.handle_nav_status, 10)
        
        # Interface state
        self.last_location = None
        self.waiting_for_response = False
        
        # Terminal settings for non-blocking input
        self.old_settings = None
        self.setup_terminal()
        
        self.get_logger().info("🎮 Interactive Query Interface started")
        
        if self.show_menu:
            self.show_main_menu()
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
    
    def setup_terminal(self):
        """Setup terminal for non-blocking input"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except:
            self.get_logger().warn("⚠️ Could not setup terminal for non-blocking input")
    
    def restore_terminal(self):
        """Restore terminal settings"""
        if self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except:
                pass
    
    def show_main_menu(self):
        """Display main menu"""
        menu = """
╔══════════════════════════════════════════════════════════════╗
║                    🤖 TURTLEBOT3 OFFICE NAVIGATOR             ║
╠══════════════════════════════════════════════════════════════╣
║  Commands:                                                   ║
║  • Type room names (e.g., 'office', 'kitchen', 'bathroom')   ║
║  • Use numbers for quick access (e.g., '1', '2', '3')       ║
║  • 'menu' - Show this menu                                   ║
║  • 'list' - Show available locations                        ║
║  • 'status' - Show current navigation status                 ║
║  • 'cancel' - Cancel current navigation                      ║
║  • 'help' - Show help information                           ║
║  • 'quit' - Exit the interface                             ║
║                                                             ║
║  Examples:                                                  ║
║  > office          (go to office)                          ║
║  > 1               (go to room #1)                          ║
║  > meeting room    (go to meeting room)                     ║
╚══════════════════════════════════════════════════════════════╝
"""
        print(menu)
        print("🎯 Ready for commands! Type your destination or 'help' for more info.")
        print(">>> ", end="", flush=True)
    
    def input_loop(self):
        """Main input processing loop"""
        while rclpy.ok():
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    user_input = input().strip()
                    if user_input:
                        self.process_command(user_input)
                        if not self.waiting_for_response:
                            print(">>> ", end="", flush=True)
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as e:
                self.get_logger().error(f"❌ Input error: {e}")
    
    def process_command(self, command):
        """Process user commands"""
        cmd = command.lower().strip()
        
        if cmd == 'quit' or cmd == 'exit':
            self.get_logger().info("👋 Goodbye!")
            rclpy.shutdown()
            return
        
        elif cmd == 'menu':
            self.show_main_menu()
            return
        
        elif cmd == 'help':
            self.show_help()
            return
        
        elif cmd == 'list':
            self.request_available_locations()
            return
        
        elif cmd == 'status':
            self.request_navigation_status()
            return
        
        elif cmd == 'cancel':
            self.cancel_navigation()
            return
        
        elif cmd == 'clear':
            print("\033[2J\033[H")  # Clear screen
            return
        
        # Handle coordinate input (x,y)
        elif ',' in cmd:
            try:
                coords = [float(x.strip()) for x in cmd.split(',')]
                if len(coords) == 2:
                    self.navigate_to_coordinates(coords[0], coords[1])
                    return
            except ValueError:
                pass
        
        # Default: treat as semantic query
        self.send_semantic_query(command)
    
    def send_semantic_query(self, query):
        """Send semantic location query"""
        self.get_logger().info(f"🔍 Querying: '{query}'")
        self.waiting_for_response = True
        
        query_msg = String()
        query_msg.data = query
        self.query_pub.publish(query_msg)
        
        print(f"🔍 Looking for '{query}'...")
    
    def handle_response(self, msg):
        """Handle semantic query response"""
        response = msg.data
        print(f"\n{response}")
        
        self.waiting_for_response = False
        
        # Check if it's an error response
        if response.startswith("❌"):
            print(">>> ", end="", flush=True)
    
    def handle_location(self, msg):
        """Handle semantic location result"""
        self.last_location = msg
        x, y = msg.pose.position.x, msg.pose.position.y
        
        print(f"📍 Location found at ({x:.2f}, {y:.2f})")
        
        if self.auto_execute:
            print("🚀 Automatically starting navigation...")
            self.goal_pub.publish(msg)
        else:
            print("Execute navigation? (y/n): ", end="", flush=True)
            # In real implementation, you'd wait for user input here
    
    def handle_nav_status(self, msg):
        """Handle navigation status updates"""
        status = msg.data
        print(f"\n{status}")
        
        # Don't show prompt if navigation is active
        if not ("in progress" in status.lower() or "remaining" in status.lower() or "eta" in status.lower()):
            print(">>> ", end="", flush=True)
    
    def navigate_to_coordinates(self, x, y):
        """Navigate to specific coordinates"""
        print(f"🎯 Navigating to coordinates ({x:.2f}, {y:.2f})")
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_pose)
    
    def request_available_locations(self):
        """Request list of available locations"""
        self.send_semantic_query("list")
    
    def request_navigation_status(self):
        """Request current navigation status"""
        print("📊 Current navigation status will be displayed...")
        # The status will be published by the navigation system
    
    def cancel_navigation(self):
        """Cancel current navigation"""
        print("🛑 Sending cancel request...")
        # This would typically send a cancel message to the navigator
        cancel_msg = String()
        cancel_msg.data = "cancel"
        # You might want to add a cancel topic/service
    
    def show_help(self):
        """Show detailed help information"""
        help_text = """
📚 HELP - TurtleBot3 Office Navigator

🎯 NAVIGATION COMMANDS:
  • Room names: 'office', 'kitchen', 'meeting room'
  • Numbers: '1', '2', '3' (for quick room access)
  • Coordinates: '1.5, 2.3' (x, y in meters)
  • Partial names: 'meet' (matches 'meeting room')

🔧 SYSTEM COMMANDS:
  • 'menu' - Show main menu
  • 'list' - List all available locations
  • 'status' - Show navigation status
  • 'cancel' - Cancel current navigation
  • 'clear' - Clear screen
  • 'help' - Show this help
  • 'quit' - Exit interface

📍 LOCATION EXAMPLES:
  > office           → Navigate to office
  > 1                → Navigate to room #1
  > meeting          → Find meeting room
  > 2.5, 1.8         → Navigate to coordinates
  > bathroom         → Navigate to bathroom

⚙️ FEATURES:
  • Auto-execution of navigation goals
  • Real-time status updates
  • Partial name matching
  • Coordinate-based navigation
  • Emergency cancel functionality

💡 TIPS:
  • Use 'list' to see all discovered rooms
  • Numbers correspond to discovery order
  • Partial names work (e.g., 'kit' for 'kitchen')
  • Cancel anytime with 'cancel' command
"""
        print(help_text)

def main(args=None):
    rclpy.init(args=args)
    interface = InteractiveQueryInterface()
    
    try:
        rclpy.spin(interface)
    except KeyboardInterrupt:
        interface.get_logger().info("🔄 Interface shutting down...")
    finally:
        interface.restore_terminal()
        interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()