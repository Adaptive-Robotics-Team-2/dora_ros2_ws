import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import csv

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Create an action client to send the waypoints to Nav2
        self._navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    
    def send_waypoints(self, waypoints):
        """
        Send a list of waypoints to the Nav2 stack, one by one, waiting for the result of each before sending the next.
        """
        for waypoint in waypoints:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.create_pose(waypoint)
            
            self.get_logger().info(f'Sending goal: {waypoint}')
            
            # Wait for the action server to be available
            self._navigate_action_client.wait_for_server()
            
            # Send the goal and wait for it to complete
            self._send_goal_and_wait(goal_msg)
    
    def read_csv(self, file_path):
        """
        Read a CSV file containing waypoints and return a list of waypoints.
        """
        waypoints = []
        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header row
            for row in reader:
                x, y = row  # Read only x and y
                waypoints.append((float(x), float(y)))  # Append the (x, y) coordinates
        return waypoints

    
    def create_pose(self, waypoint):
        """
        Create a PoseStamped message from a waypoint (x, y) without orientation.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        # Set the position (x, y)
        pose_stamped.pose.position.x = waypoint[0]
        pose_stamped.pose.position.y = waypoint[1]

        # Set orientation to default (facing straight ahead, i.e., yaw = 0.0)
        pose_stamped.pose.orientation.w = 1.0  # Quaternion for no rotation (yaw = 0.0)
        
        return pose_stamped

    def _send_goal_and_wait(self, goal_msg):
        """
        Send a goal to the action server and wait for the result before proceeding to the next goal.
        """
        send_goal_future = self._navigate_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        # Wait for the navigation goal to complete
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle and goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            if result:
                self.get_logger().info(f'Navigation to goal {goal_msg.pose} completed successfully.')
            else:
                self.get_logger().info(f'Navigation to goal {goal_msg.pose} failed.')
        else:
            self.get_logger().info('Goal was rejected.')

    def goal_response_callback(self, future):
        """
        Handle the response from the action server.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected.')
        else:
            self.get_logger().info('Goal accepted.')

def main(args=None):
    rclpy.init(args=args)

    # Initialize the WaypointNavigator node
    navigator = WaypointNavigator()

    # Path to your CSV file containing the waypoints
    csv_file_path = 'Waypoints.csv'

    # Read waypoints from CSV file
    waypoints = navigator.read_csv(csv_file_path)

    # Send the waypoints to Nav2 for navigation
    navigator.send_waypoints(waypoints)

    # Spin the node until execution completes
    rclpy.spin(navigator)

    # Clean up
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
