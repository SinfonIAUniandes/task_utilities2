#!/usr/bin/env python3

"""
Miscellaneous Proxy for TaskModule

This module provides miscellaneous robot functionality through ROS2 services and topics.
It exposes the capabilities of the naoqi_miscellaneous_node including:
- Autonomous life control
- Basic awareness management  
- Autonomous blinking control
- LED control
- Battery monitoring

It follows the same pattern as SpeechProxy, using a parent node instead of creating its own.
"""

# Import required service and message types
from std_srvs.srv import SetBool
from std_msgs.msg import Float32
from naoqi_utilities_msgs.msg import LedParameters
from std_srvs.srv import SetBool

class MiscellaneousProxy:
    """
    Miscellaneous proxy that uses a parent node for ROS2 communication.
    This allows TaskModule to be the single ROS2 node managing all services.
    """
    
    def __init__(self, parent_node, miscellaneous_node_name="naoqi_miscellaneous_node"):
        """
        Initialize Miscellaneous proxy with a parent node.
        
        Args:
            parent_node (Node): The parent ROS2 node to use for services
            miscellaneous_node_name (str): Name of the miscellaneous node
        """
        self.node = parent_node
        self.miscellaneous_node_name = miscellaneous_node_name
        
        # Create service clients using the parent node
        self.set_autonomous_state_client = self.node.create_client(
            SetBool, 
            f"/{miscellaneous_node_name}/set_autonomous_state"
        )
        self.toggle_awareness_client = self.node.create_client(
            SetBool, 
            f"/{miscellaneous_node_name}/toggle_awareness"
        )
        self.toggle_blinking_client = self.node.create_client(
            SetBool, 
            f"/{miscellaneous_node_name}/toggle_blinking"
        )
        
        # Create publisher for LED control
        self.leds_publisher = self.node.create_publisher(
            LedParameters, 
            '/set_leds', 
            10
        )
        
        # Battery monitoring
        self.battery_percentage = None
        self.battery_subscription = self.node.create_subscription(
            Float32,
            '/battery_percentage',
            self._battery_callback,
            10
        )
        
        # Wait for services to be available
        self.wait_for_services()
        
        self.node.get_logger().info("MiscellaneousProxy initialized")
    
    def wait_for_services(self):
        """Wait for all miscellaneous services to become available."""
        self.node.get_logger().info("Waiting for miscellaneous services...")
        
        services = [
            (self.set_autonomous_state_client, 'set_autonomous_state'),
            (self.toggle_awareness_client, 'toggle_awareness'),
            (self.toggle_blinking_client, 'toggle_blinking')
        ]
        
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(f'{service_name} service not available, waiting again...')
        
        self.node.get_logger().info("All miscellaneous services are available")
    
    def _battery_callback(self, msg):
        """Internal callback to store battery percentage."""
        self.battery_percentage = msg.data
    
    # Autonomous Life Methods
    def set_autonomous_state(self, enabled: bool) -> bool:
        """
        Enable or disable the robot's autonomous life mode.
        
        When enabled, the robot enters "interactive" mode and can perform
        autonomous behaviors. When disabled, the robot goes to "disabled" 
        state and assumes a standing posture.
        
        Args:
            enabled (bool): True to enable autonomous life, False to disable
            
        Returns:
            bool: True if successful, False otherwise
        """
        
        state_str = "enable" if enabled else "disable"
        self.node.get_logger().info(f"Setting autonomous state to {state_str}...")
        request = SetBool.Request()
        request.data = enabled

        response = self.set_autonomous_state_client.call(request)
        return response.success
    
    # Basic Awareness Methods
    def toggle_awareness(self, enabled: bool) -> bool:
        """
        Enable or disable the robot's basic awareness.
        
        Basic awareness controls head movements in response to stimuli.
        When enabled, the robot will track people and objects with its head.
        
        Args:
            enabled (bool): True to enable awareness, False to disable
            
        Returns:
            bool: True if successful, False otherwise
        """
        
        state_str = "enable" if enabled else "disable"
        self.node.get_logger().info(f"Setting basic awareness to {state_str}...")
        request = SetBool.Request()
        request.data = enabled
        
        response = self.toggle_awareness_client.call(request)
        return response.success
    
    # Autonomous Blinking Methods
    def toggle_blinking(self, enabled: bool) -> bool:
        """
        Enable or disable the robot's autonomous blinking.
        
        When enabled, the robot's eye LEDs will blink naturally.
        When disabled, the eyes remain in their current state.
        
        Args:
            enabled (bool): True to enable blinking, False to disable
            
        Returns:
            bool: True if successful, False otherwise
        """
        
        state_str = "enable" if enabled else "disable"
        self.node.get_logger().info(f"Setting autonomous blinking to {state_str}...")
        request = SetBool.Request()
        request.data = enabled
        
        response = self.toggle_blinking_client.call(request)
        return response.success
    
    # LED Control Methods
    def set_leds(self, name: str, red: int = 0, green: int = 0, blue: int = 0, duration: float = 0.0) -> bool:
        """
        Set the color of an LED or LED group.
        
        Args:
            name (str): Name of the LED or LED group (e.g., "FaceLeds", "EyeLeds", "LeftEar")
            red (int): Red component (0-255)
            green (int): Green component (0-255)
            blue (int): Blue component (0-255)
            duration (float): Fade duration in seconds (0.0 for instant change)
            
        Returns:
            bool: True if message was published successfully
        """
        try:
            
            # Validate color values
            red = max(0, min(255, int(red)))
            green = max(0, min(255, int(green)))
            blue = max(0, min(255, int(blue)))
            
            self.node.get_logger().info(
                f"Setting LED '{name}' to RGB({red}, {green}, {blue}) over {duration}s"
            )
            
            msg = LedParameters()
            msg.name = name
            msg.red = red
            msg.green = green
            msg.blue = blue
            msg.duration = float(duration)
            
            self.leds_publisher.publish(msg)
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to set LEDs: {e}")
            return False