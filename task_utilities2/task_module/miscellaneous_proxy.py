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
        
        if response and response.success:
            self.node.get_logger().info(f"Autonomous state changed: {response.message}")
            return True
        else:
            error_msg = response.message if response else "No response from service"
            self.node.get_logger().error(f"Failed to change autonomous state: {error_msg}")
            return False
    
    def enable_autonomous_life(self) -> bool:
        """
        Enable the robot's autonomous life mode.
        
        Returns:
            bool: True if successful, False otherwise
        """
        return self.set_autonomous_state(True)
    
    def disable_autonomous_life(self) -> bool:
        """
        Disable the robot's autonomous life mode.
        
        Returns:
            bool: True if successful, False otherwise
        """
        return self.set_autonomous_state(False)
    
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
        
        if response and response.success:
            self.node.get_logger().info(f"Basic awareness changed: {response.message}")
            return True
        else:
            error_msg = response.message if response else "No response from service"
            self.node.get_logger().error(f"Failed to change basic awareness: {error_msg}")
            return False
    
    def enable_awareness(self) -> bool:
        """
        Enable the robot's basic awareness.
        
        Returns:
            bool: True if successful, False otherwise
        """
        return self.toggle_awareness(True)
    
    def disable_awareness(self) -> bool:
        """
        Disable the robot's basic awareness.
        
        Returns:
            bool: True if successful, False otherwise
        """
        return self.toggle_awareness(False)
    
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
        
        if response and response.success:
            self.node.get_logger().info(f"Autonomous blinking changed: {response.message}")
            return True
        else:
            error_msg = response.message if response else "No response from service"
            self.node.get_logger().error(f"Failed to change blinking state: {error_msg}")
            return False
    
    def enable_blinking(self) -> bool:
        """
        Enable the robot's autonomous blinking.
        
        Returns:
            bool: True if successful, False otherwise
        """
        return self.toggle_blinking(True)
    
    def disable_blinking(self) -> bool:
        """
        Disable the robot's autonomous blinking.
        
        Returns:
            bool: True if successful, False otherwise
        """
        return self.toggle_blinking(False)
    
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
    
    def set_eye_color(self, red: int = 0, green: int = 0, blue: int = 0, duration: float = 0.0) -> bool:
        """
        Set the color of the robot's eyes.
        
        Args:
            red (int): Red component (0-255)
            green (int): Green component (0-255)
            blue (int): Blue component (0-255)
            duration (float): Fade duration in seconds
            
        Returns:
            bool: True if successful
        """
        return self.set_leds("FaceLeds", red, green, blue, duration)
    
    def set_ear_color(self, ear: str, red: int = 0, green: int = 0, blue: int = 0, duration: float = 0.0) -> bool:
        """
        Set the color of a specific ear.
        
        Args:
            ear (str): "Left" or "Right"
            red (int): Red component (0-255)
            green (int): Green component (0-255)
            blue (int): Blue component (0-255)
            duration (float): Fade duration in seconds
            
        Returns:
            bool: True if successful
        """
        ear_name = f"{ear}Ear" if ear in ["Left", "Right"] else ear
        return self.set_leds(ear_name, red, green, blue, duration)
    
    def turn_off_leds(self, led_group: str = "AllLeds", duration: float = 1.0) -> bool:
        """
        Turn off a group of LEDs.
        
        Args:
            led_group (str): LED group name (default: "AllLeds")
            duration (float): Fade out duration in seconds
            
        Returns:
            bool: True if successful
        """
        return self.set_leds(led_group, 0, 0, 0, duration)
    
    # Convenience LED Color Methods
    def set_eyes_red(self, duration: float = 0.5) -> bool:
        """Set eyes to red color."""
        return self.set_eye_color(255, 0, 0, duration)
    
    def set_eyes_green(self, duration: float = 0.5) -> bool:
        """Set eyes to green color."""
        return self.set_eye_color(0, 255, 0, duration)
    
    def set_eyes_blue(self, duration: float = 0.5) -> bool:
        """Set eyes to blue color."""
        return self.set_eye_color(0, 0, 255, duration)
    
    def set_eyes_white(self, duration: float = 0.5) -> bool:
        """Set eyes to white color."""
        return self.set_eye_color(255, 255, 255, duration)
    
    def set_eyes_yellow(self, duration: float = 0.5) -> bool:
        """Set eyes to yellow color."""
        return self.set_eye_color(255, 255, 0, duration)
    
    def set_eyes_purple(self, duration: float = 0.5) -> bool:
        """Set eyes to purple color."""
        return self.set_eye_color(128, 0, 128, duration)
    
    def set_eyes_cyan(self, duration: float = 0.5) -> bool:
        """Set eyes to cyan color."""
        return self.set_eye_color(0, 255, 255, duration)
    
    # Battery Monitoring Methods
    def get_battery_percentage(self) -> float | None:
        """
        Get the current battery percentage.
        
        Returns:
            float: Battery percentage (0-100), or None if not available
        """
        if self.battery_percentage is not None:
            self.node.get_logger().info(f"Battery level: {self.battery_percentage}%")
        else:
            self.node.get_logger().warn("Battery percentage not available yet")
        
        return self.battery_percentage
    
    def is_battery_low(self, threshold: float = 20.0) -> bool:
        """
        Check if battery is below a certain threshold.
        
        Args:
            threshold (float): Battery percentage threshold (default: 20%)
            
        Returns:
            bool: True if battery is low, False otherwise (or if battery level unknown)
        """
        battery_level = self.get_battery_percentage()
        if battery_level is not None:
            return battery_level < threshold
        return False
    
    def is_battery_critical(self, threshold: float = 10.0) -> bool:
        """
        Check if battery is critically low.
        
        Args:
            threshold (float): Battery percentage threshold (default: 10%)
            
        Returns:
            bool: True if battery is critically low, False otherwise
        """
        return self.is_battery_low(threshold)
    
    # Utility Methods
    def setup_default_state(self) -> bool:
        """
        Set up the robot with default miscellaneous settings.
        
        This enables autonomous blinking and basic awareness for natural behavior.
        
        Returns:
            bool: True if all settings were applied successfully
        """
        self.node.get_logger().info("Setting up default miscellaneous state...")
        
        success = True
        
        # Enable autonomous blinking for natural eye behavior
        if not self.enable_blinking():
            success = False
            
        # Enable basic awareness for natural head movements
        if not self.enable_awareness():
            success = False
        
        # Set eyes to a natural white color
        if not self.set_eyes_white(duration=1.0):
            success = False
        
        if success:
            self.node.get_logger().info("Default miscellaneous state setup completed successfully")
        else:
            self.node.get_logger().warn("Some default settings failed to apply")
            
        return success
    
    def shutdown(self) -> bool:
        """
        Shutdown miscellaneous systems gracefully.
        
        This turns off LEDs and disables autonomous behaviors.
        
        Returns:
            bool: True if shutdown was successful
        """
        self.node.get_logger().info("Shutting down miscellaneous systems...")
        
        success = True
        
        # Turn off all LEDs
        if not self.turn_off_leds(duration=1.0):
            success = False
        
        # Disable autonomous behaviors
        if not self.disable_awareness():
            success = False
            
        if not self.disable_blinking():
            success = False
        
        if success:
            self.node.get_logger().info("Miscellaneous systems shutdown completed")
        else:
            self.node.get_logger().warn("Some shutdown operations failed")
            
        return success