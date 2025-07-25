# ----------------code with timeout-------------
# #!/usr/bin/env python3
# import os
# import yaml
# import cv2
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from nav2_msgs.action import NavigateToPose
# from rclpy.action import ActionClient
# from geometry_msgs.msg import PoseStamped

# class CoverageNavigator(Node):
#     def __init__(self):
#         super().__init__('coverage_navigator')
#         self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
#         self.waypoints = []
#         self.current_waypoint = 0
#         self.timeout_duration = 30.0  # seconds
#         self.goal_timer = None

#         self.load_map_and_generate_path()

#         if self.waypoints:
#             self.send_next_waypoint()

#     def load_map_and_generate_path(self):
#         try:
#             map_yaml_path = os.path.expanduser('/home/samah/robot_ws/src/map_server/config/room_area.yaml')
#             robot_size = 0.4  # meters
#             overlap = 0.1
#             inflation = 0.2  # meters to keep away from walls

#             with open(map_yaml_path, 'r') as f:
#                 map_data = yaml.safe_load(f)

#             map_img_path = os.path.join(os.path.dirname(map_yaml_path), map_data['image'])
#             map_img = cv2.imread(map_img_path, cv2.IMREAD_GRAYSCALE)
#             if map_img is None:
#                 self.get_logger().error("Failed to load map image!")
#                 return

#             height, width = map_img.shape
#             resolution = map_data['resolution']
#             origin_x = map_data['origin'][0]
#             origin_y = map_data['origin'][1]

#             # Inflate walls
#             obstacle_mask = (map_img < 250).astype(np.uint8)
#             inflation_radius = int(inflation / resolution)
#             kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * inflation_radius, 2 * inflation_radius))
#             inflated_obstacles = cv2.dilate(obstacle_mask, kernel)

#             free_space = cv2.bitwise_and((map_img > 250).astype(np.uint8), cv2.bitwise_not(inflated_obstacles))

#             step_pixels = int((robot_size - overlap) / resolution)
#             contours, _ = cv2.findContours(free_space, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#             if contours:
#                 x, y, w, h = cv2.boundingRect(contours[0])
#                 direction = 1
#                 current_y = y + step_pixels

#                 while current_y < y + h:
#                     if direction == 1:
#                         start_x, end_x = x, x + w
#                     else:
#                         start_x, end_x = x + w, x

#                     # Find first and last valid pixel in this line
#                     line_points = []
#                     for current_x in range(start_x, end_x, step_pixels * direction):
#                         if free_space[current_y, current_x]:
#                             line_points.append((current_x, current_y))

#                     if line_points:
#                         first_px, first_py = line_points[0]
#                         last_px, last_py = line_points[-1]

#                         for px, py in [(first_px, first_py), (last_px, last_py)]:
#                             world_x = origin_x + (px * resolution)
#                             world_y = origin_y + ((height - py) * resolution)
#                             self.waypoints.append((world_x, world_y))

#                     current_y += step_pixels
#                     direction *= -1

#                 self.get_logger().info(f"Generated {len(self.waypoints)} optimized waypoints")

#         except Exception as e:
#             self.get_logger().error(f"Error during map loading: {str(e)}")

#     def send_next_waypoint(self):
#         if self.current_waypoint >= len(self.waypoints):
#             self.get_logger().info("All waypoints completed!")
#             return

#         x, y = self.waypoints[self.current_waypoint]
#         self.get_logger().info(f"Navigating to waypoint {self.current_waypoint+1}/{len(self.waypoints)}: x={x:.2f}, y={y:.2f}")

#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = PoseStamped()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.pose.position.x = x
#         goal_msg.pose.pose.position.y = y
#         goal_msg.pose.pose.orientation.w = 1.0

#         self.nav_client.wait_for_server()
#         self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
#         self.send_goal_future.add_done_callback(self.goal_response_callback)

#         # Start timeout timer
#         if self.goal_timer:
#             self.goal_timer.cancel()
#         self.goal_timer = self.create_timer(self.timeout_duration, self.goal_timeout_handler)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             if self.goal_timer:
#                 self.goal_timer.cancel()
#             self.get_logger().error("Waypoint rejected! Moving to next...")
#             self.current_waypoint += 1
#             self.send_next_waypoint()
#             return

#         self.get_logger().info("Waypoint accepted")
#         self.result_future = goal_handle.get_result_async()
#         self.result_future.add_done_callback(self.navigation_result_callback)

#     def navigation_result_callback(self, future):
#         if self.goal_timer:
#             self.goal_timer.cancel()

#         result = future.result().result
#         self.get_logger().info(f"Navigation result: {result}")
#         self.current_waypoint += 1
#         self.send_next_waypoint()

#     def goal_timeout_handler(self):
#         self.get_logger().warn(f"Goal {self.current_waypoint+1} timed out! Skipping...")
#         if hasattr(self, 'result_future') and not self.result_future.done():
#             self.result_future.cancel()
#         if self.goal_timer:
#             self.goal_timer.cancel()
#         self.current_waypoint += 1
#         self.send_next_waypoint()

# def main(args=None):
#     rclpy.init(args=args)
#     navigator = CoverageNavigator()
#     rclpy.spin(navigator)
#     navigator.destroy_node()
#     rclpy.shutdown()


# ----------------------code without time out------------
#!/usr/bin/env python3
import os
import yaml
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class CoverageNavigator(Node):
    def __init__(self):
        super().__init__('coverage_navigator')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.waypoints = []
        self.current_waypoint = 0
        
        # Load map and generate path
        self.load_map_and_generate_path()
        
        # Start navigation if waypoints exist
        if self.waypoints:
            self.send_next_waypoint()

    def load_map_and_generate_path(self):
        """Load map and generate sparse zigzag path"""
        try:
            map_yaml_path = os.path.expanduser('/home/samah/robot_ws/src/map_server/config/room_area.yaml')
            robot_size = 0.4  # meters
            overlap = 0.1     # meters

            with open(map_yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)

            map_img_path = os.path.join(os.path.dirname(map_yaml_path), map_data['image'])
            map_img = cv2.imread(map_img_path, cv2.IMREAD_GRAYSCALE)

            if map_img is None:
                self.get_logger().error("Failed to load map image!")
                return

            height, width = map_img.shape
            resolution = map_data['resolution']
            origin_x = map_data['origin'][0]
            origin_y = map_data['origin'][1]
            step_px = int((robot_size - overlap) / resolution)

            _, bin_map = cv2.threshold(map_img, 250, 255, cv2.THRESH_BINARY)
            bin_map = bin_map // 255  # to 0/1
              
             # === DILATE OBSTACLES FOR 30cm BUFFER ===
            dilation_distance_m = 0.3  # 30 cm
            dilation_radius_px = int(dilation_distance_m / resolution)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilation_radius_px + 1, 2 * dilation_radius_px + 1))
            dilated_obstacles = cv2.dilate(1 - bin_map, kernel)  # Dilate obstacles (invert first)
            bin_map = 1 - dilated_obstacles  # Invert back: 1 = free, 0 = obstacle

              


            direction = 1
            for row in range(0, height, step_px):
                cols = range(0, width) if direction == 1 else range(width - 1, -1, -1)
                valid_cols = [col for col in cols if bin_map[row, col] == 1]

                if valid_cols:
                    start_col = valid_cols[0]
                    end_col = valid_cols[-1]

                    for col in [start_col, end_col]:
                        world_x = origin_x + (col * resolution)
                        world_y = origin_y + ((height - row) * resolution)
                        self.waypoints.append((world_x, world_y))

                direction *= -1

            self.get_logger().info(f"Generated {len(self.waypoints)} optimized waypoints")

        except Exception as e:
            self.get_logger().error(f"Error during map loading: {str(e)}")

    def send_next_waypoint(self):
        """Send next waypoint to navigation stack"""
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info("All waypoints completed!")
            return

        x, y = self.waypoints[self.current_waypoint]
        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint+1}/{len(self.waypoints)}: x={x:.2f}, y={y:.2f}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # No rotation for simplicity

        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Waypoint rejected! Moving to next...")
            self.current_waypoint += 1
            self.send_next_waypoint()
            return

        self.get_logger().info("Waypoint accepted")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")
        self.current_waypoint += 1
        self.send_next_waypoint()

def main(args=None):
    rclpy.init(args=args)
    navigator = CoverageNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()
















# import rclpy
# from nav2_msgs.action import NavigateToPose
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped

# class NavToPoseClient(Node):
#     def __init__(self):
#         super().__init__('nav_to_pose_client')
#         self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
#         self.get_logger().info("Action client created. Call send_goal() to navigate.")

#     def send_goal(self, x, y):
#         # Create goal message (identical to your working CLI command)
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = PoseStamped()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.pose.position.x = x
#         goal_msg.pose.pose.position.y = y
#         goal_msg.pose.pose.orientation.w = 1.0  # Default orientation

#         self.get_logger().info(f"Sending goal: x={x}, y={y}")
#         self._action_client.wait_for_server()
#         self._send_goal_future = self._action_client.send_goal_async(goal_msg)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Goal rejected')
#             return
#         self.get_logger().info('Goal accepted!')

# def main(args=None):
#     rclpy.init(args=args)
#     client = NavToPoseClient()
    
#     # Hardcoded goal (same coordinates that worked in CLI)
#     client.send_goal(x=1.0, y=0.0)  # Change these values as needed
    
#     rclpy.spin(client)
#     client.destroy_node()
#     rclpy.shutdown()



