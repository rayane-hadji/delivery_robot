#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String, Header
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
import time


class DeliveryRobot(Node):
    def __init__(self):
        super().__init__('delivery_robot')

        # --- Nav2 action client ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- Subscriber for client orders ---
        self.create_subscription(String, 'client_order', self.order_callback, 10)

        # --- Zones with proper header setup ---
        self.zones = {
            "green": PoseStamped(
                header=Header(frame_id="map"),
                pose=Pose(
                    position=Point(x=0.6955609321594238, y=-0.07421714067459106, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0006664271112781355, w=0.999999777937428)
                )
            ),
            "blue": PoseStamped(
                header=Header(frame_id="map"), 
                pose=Pose(
                    position=Point(x=0.5072014331817627, y=3.1744954586029053, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.11238287105418615, w=0.9936649789006444)
                )
            ),
            "yellow": PoseStamped(
                header=Header(frame_id="map"),
                pose=Pose(
                    position=Point(x=0.5003886222839355, y=-3.259990692138672, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0014715618464396498, w=0.9999989172522799)
                )
            ),
            "red": PoseStamped(
                header=Header(frame_id="map"),
                pose=Pose(
                    position=Point(x=-9.022370338439941, y=10.41213607788086, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.15414861020243525, w=0.9880476739371727)
                )
            ),
            "orange": PoseStamped(
                header=Header(frame_id="map"),
                pose=Pose(
                    position=Point(x=0.8442726135253906, y=-16.447877883911133, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=-0.04345594183183841, w=0.9990553443726268)
                )
            ),
            "white": PoseStamped(
                header=Header(frame_id="map"),
                pose=Pose(
                    position=Point(x=12.19839859008789, y=-3.225337028503418, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=-0.026647385108634583, w=0.9996448953837919)
                )
            )
        }

        self.current_task_active = False
        self.current_goal_handle = None
        self.wait_timer = None
        
        self.get_logger().info("✅ Delivery robot ready. Publish order to /client_order (e.g., 'blue->orange')")

    def order_callback(self, msg):
        if self.current_task_active:
            self.get_logger().warn("⚠️ A delivery is already in progress. Please wait.")
            return

        try:
            pickup, delivery = msg.data.split('->')
            pickup, delivery = pickup.strip().lower(), delivery.strip().lower()

            if pickup not in self.zones or delivery not in self.zones:
                raise ValueError("Unknown zone name.")

            self.get_logger().info(f"🟢 Order received: {pickup} → {delivery}")
            self.current_task_active = True
            
            # Start delivery sequence
            self.execute_delivery_sequence(pickup, delivery)

        except Exception as e:
            self.get_logger().error(f"❌ Invalid command format or unknown zone: {e}")

    def execute_delivery_sequence(self, pickup, delivery):
        """Execute the complete delivery sequence asynchronously"""
        self.get_logger().info("🚀 Starting delivery sequence...")
        
        # Store delivery parameters
        self.delivery_pickup = pickup
        self.delivery_destination = delivery
        
        # Start with pickup zone
        self.navigate_to_zone_async(pickup, "pickup")

    def navigate_to_zone_async(self, zone_name, stage):
        """Navigate to a zone asynchronously"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Action server not available!")
            self.current_task_active = False
            return

        goal_pose = self.zones[zone_name]
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"🚀 Navigating to {zone_name} zone...")

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, zone_name, stage)
        )

    def goal_response_callback(self, future, zone_name, stage):
        """Handle the response from the action server"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"❌ Goal to {zone_name} rejected.")
                self.current_task_active = False
                return

            self.get_logger().info(f"✅ Goal to {zone_name} accepted!")
            self.current_goal_handle = goal_handle

            # Get result
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(
                lambda future: self.get_result_callback(future, zone_name, stage)
            )
        except Exception as e:
            self.get_logger().error(f"❌ Error in goal response: {e}")
            self.current_task_active = False

    def get_result_callback(self, future, zone_name, stage):
        """Handle the result from navigation"""
        try:
            result = future.result()
            status = result.status
            
            # Check if goal was successful (STATUS_SUCCEEDED = 4)
            if status == 4:
                self.get_logger().info(f"✅ Successfully reached {zone_name} zone!")
                
                # Add a small delay to ensure robot fully stops
                self.wait_timer = self.create_timer(2.0, lambda: self.after_stop_delay(zone_name, stage))
            else:
                self.get_logger().error(f"❌ Failed to reach {zone_name} zone. Status: {status}")
                self.current_task_active = False
        except Exception as e:
            self.get_logger().error(f"❌ Error in result callback: {e}")
            self.current_task_active = False

    def after_stop_delay(self, zone_name, stage):
        """Wait a moment after reaching to ensure complete stop"""
        if self.wait_timer:
            self.wait_timer.cancel()
            self.wait_timer = None
            
        self.get_logger().info("🛑 Robot fully stopped")
        self.handle_navigation_success(zone_name, stage)

    def handle_navigation_success(self, zone_name, stage):
        """Handle what to do after successfully reaching a zone"""
        if stage == "pickup":
            self.get_logger().info("📦 Reached pickup zone. Waiting 5 seconds...")
            # Wait 5 seconds then go to delivery
            self.wait_timer = self.create_timer(5.0, self.start_delivery_phase)
            
        elif stage == "delivery":
            self.get_logger().info("✅ Package delivered. Waiting 5 seconds...")
            # Wait 5 seconds then return to green
            self.wait_timer = self.create_timer(5.0, self.start_return_phase)
            
        elif stage == "return":
            self.get_logger().info("🏁 Returned to green zone. Delivery complete!")
            self.get_logger().info("🛑 Robot is now completely stopped and waiting for next command")
            self.current_task_active = False
            self.get_logger().info("✅ Delivery cycle complete. Ready for next order.\n")

    def start_delivery_phase(self):
        """Start navigation to delivery zone"""
        if self.wait_timer:
            self.wait_timer.cancel()
            self.wait_timer = None
        self.navigate_to_zone_async(self.delivery_destination, "delivery")

    def start_return_phase(self):
        """Start navigation back to green zone"""
        if self.wait_timer:
            self.wait_timer.cancel()
            self.wait_timer = None
        self.navigate_to_zone_async("green", "return")

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # Optional: Uncomment to see distance remaining
        # self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f}m", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryRobot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Delivery robot shutdown by user")
    finally:
        # Cleanup
        if node.wait_timer:
            node.wait_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()