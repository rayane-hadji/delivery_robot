#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify
from threading import Thread
import json
import time

class ROSWebBridge(Node):
    def __init__(self):
        super().__init__('ros_web_bridge')
        
        # Publisher for commands - same topic your delivery robot listens to
        self.command_publisher = self.create_publisher(String, '/client_order', 10)
        
        # Track delivery status
        self.delivery_status = "ready"  # ready, busy, error
        
        self.get_logger().info("🌐 ROS Web Bridge started - Waiting for commands...")

    def publish_command(self, command):
        """Publish a command to the ROS topic"""
        try:
            # Validate command format
            if '->' not in command:
                return False, "Invalid command format. Use 'pickup_zone->delivery_zone'"
            
            pickup, delivery = command.split('->')
            pickup = pickup.strip().lower()
            delivery = delivery.strip().lower()
            
            valid_zones = ["green", "blue", "yellow", "red", "orange", "white"]
            if pickup not in valid_zones or delivery not in valid_zones:
                return False, f"Invalid zone name. Available zones: {valid_zones}"
            
            msg = String()
            msg.data = command
            self.command_publisher.publish(msg)
            self.get_logger().info(f"📤 Published command: {command}")
            
            # Update status to indicate robot is busy
            self.delivery_status = "busy"
            
            # Schedule status reset after reasonable time (adjust based on your robot's typical delivery time)
            # This is a simple approach - you might want more sophisticated status tracking
            Thread(target=self._reset_status_after_delay, daemon=True).start()
            
            return True, "Command sent successfully"
            
        except Exception as e:
            self.get_logger().error(f"❌ Error publishing command: {e}")
            return False, str(e)
    
    def _reset_status_after_delay(self):
        """Reset status to ready after a delay (simplified approach)"""
        time.sleep(60)  # Adjust based on typical delivery duration
        if self.delivery_status == "busy":
            self.delivery_status = "ready"
            self.get_logger().info("🔄 Status reset to 'ready'")

    def set_delivery_status(self, status):
        """Allow external status updates (could be called from your delivery robot)"""
        self.delivery_status = status

# Global variable for ROS node
ros_node = None

# Flask Web Server
app = Flask(__name__)

@app.route('/send_command', methods=['POST'])
def send_command():
    """Endpoint to receive commands from n8n/web interface"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({"status": "error", "message": "No JSON data received"}), 400
        
        command = data.get('command', '').strip()
        
        if not command:
            return jsonify({"status": "error", "message": "No command provided"}), 400
        
        if ros_node:
            # Check if robot is busy
            if ros_node.delivery_status == "busy":
                return jsonify({
                    "status": "error", 
                    "message": "Robot is currently busy with another delivery. Please try again later."
                }), 423  # 423 Locked status code
            
            success, message = ros_node.publish_command(command)
            
            if success:
                return jsonify({
                    "status": "success", 
                    "message": f"Command '{command}' sent to delivery robot",
                    "robot_status": "busy"
                })
            else:
                return jsonify({"status": "error", "message": message}), 400
        else:
            return jsonify({"status": "error", "message": "ROS node not initialized"}), 500
            
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        "status": "healthy", 
        "service": "ros_web_bridge",
        "ros_connected": ros_node is not None,
        "robot_status": ros_node.delivery_status if ros_node else "unknown"
    })

@app.route('/zones', methods=['GET'])
def get_zones():
    """Get available zones (for reference)"""
    zones = ["green", "blue", "yellow", "red", "orange", "white"]
    return jsonify({
        "available_zones": zones,
        "command_format": "pickup_zone->delivery_zone",
        "examples": ["green->blue", "red->orange", "yellow->white"]
    })

@app.route('/status', methods=['GET'])
def get_status():
    """Get current robot status"""
    if ros_node:
        return jsonify({
            "robot_status": ros_node.delivery_status,
            "ros_connected": True
        })
    else:
        return jsonify({
            "robot_status": "unknown",
            "ros_connected": False
        }), 503

def run_flask():
    """Run Flask server in a separate thread"""
    print("🚀 Starting Flask server on http://0.0.0.0:8080")
    app.run(host='0.0.0.0', port=8080, debug=False, use_reloader=False)

def main():
    global ros_node
    
    # Initialize ROS
    rclpy.init()
    
    # Create ROS node
    ros_node = ROSWebBridge()
    
    print("✅ ROS Web Bridge initialized!")
    print("📡 Available endpoints:")
    print("   POST http://localhost:8080/send_command")
    print("   GET  http://localhost:8080/health")
    print("   GET  http://localhost:8080/zones")
    print("   GET  http://localhost:8080/status")
    print("\n💡 Command format: 'green->blue' (pickup->delivery)")
    
    # 🚀 CRITICAL FIX: Start Flask in a separate thread
    flask_thread = Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    try:
        # Keep ROS running
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down ROS Web Bridge...")
    finally:
        if ros_node:
            ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()