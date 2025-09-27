#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from threading import Thread
import os

# Import proxy classes
from .speech_proxy import SpeechProxy
from .navigation_proxy import NavigationProxy
from .miscellaneous_proxy import MiscellaneousProxy


class TaskModule(Node):
    """
    Unified interface for all robot functionalities and proxies.
    
    This class serves as the main node that coordinates and provides access to
    all robot capabilities through various proxy classes. It acts as a single
    entry point for complex robot tasks that may involve multiple subsystems.
    
    Usage:
        task_module = TaskModule()
        task_module.speech.say("Hello World")
        response = task_module.speech.ask("What is your name?")
    """
    
    def __init__(self, 
                 node_name="task_module",
                 microphone_node_name="microphone_node",
                 conversation_node_name="conversation_node",
                 manipulation_node_name="naoqi_manipulation_node",
                 robot_name=None):
        """
        Initialize the TaskModule with all proxy systems.
        
        Args:
            node_name (str): Name for this ROS2 node
            microphone_node_name (str): Name of the microphone node for speech services
            conversation_node_name (str): Name of the conversation node for LLM services
            manipulation_node_name (str): Name of the manipulation node for robot control
            robot_name (str): Name of the robot (optional)
        """
        super().__init__(node_name)
        
        self.robot_name = robot_name or "robot"
        self.microphone_node_name = microphone_node_name
        self.conversation_node_name = conversation_node_name
        self.manipulation_node_name = manipulation_node_name
        
        self.get_logger().info(f"Initializing TaskModule for robot: {self.robot_name}")
        
        # Initialize proxy systems
        self._initialize_proxies()
        
        self.get_logger().info("TaskModule initialization complete")
    
    def _initialize_proxies(self):
        """Initialize all proxy systems."""
        self.get_logger().info("Initializing proxy systems...")
        
        # Initialize Speech proxy - but modify it to not create its own node
        self._initialize_speech_proxy()
        
        # Initialize Navigation proxy (when implemented)
        self._initialize_navigation_proxy()
        
        # Initialize Miscellaneous proxy
        self._initialize_miscellaneous_proxy()
        
        # Add more proxies here in the future
        # self._initialize_vision_proxy()
        # self._initialize_manipulation_proxy()
        
    def _initialize_speech_proxy(self):
        """Initialize the speech proxy system."""
        self.get_logger().info("Initializing Speech proxy...")
        
        # Create a modified Speech class that uses this node instead of creating its own
        self.speech = SpeechProxy(
            parent_node=self,
            microphone_node_name=self.microphone_node_name,
            conversation_node_name=self.conversation_node_name
        )
        
        self.get_logger().info("Speech proxy initialized")
    
    def _initialize_navigation_proxy(self):
        """Initialize the navigation proxy system."""
        self.get_logger().info("Initializing Navigation proxy...")
        
        # Initialize NavigationProxy (currently a placeholder)
        self.navigation = NavigationProxy(parent_node=self)
        
        self.get_logger().info("Navigation proxy initialized")
    
    def _initialize_miscellaneous_proxy(self):
        """Initialize the miscellaneous proxy system."""
        self.get_logger().info("Initializing Miscellaneous proxy...")
        
        # Initialize MiscellaneousProxy with manipulation node name
        self.miscellaneous = MiscellaneousProxy(
            parent_node=self,
            manipulation_node_name=self.manipulation_node_name
        )
        
        self.get_logger().info("Miscellaneous proxy initialized")
    
    def load_context_from_file(self, context_file_path):
        """
        Load context for LLM from a file.
        
        Args:
            context_file_path (str): Path to the context file
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            with open(context_file_path, 'r') as file:
                context = file.read()
                return self.speech.set_llm_settings(context=context)
        except FileNotFoundError:
            self.get_logger().error(f"Context file not found at {context_file_path}")
            return False
        except Exception as e:
            self.get_logger().error(f"Error loading context file: {e}")
            return False
    
    def load_robot_context(self, robot_name=None):
        """
        Load context for a specific robot from the data directory.
        
        Args:
            robot_name (str): Name of the robot (nova, opera, orion). 
                            If None, uses self.robot_name
            
        Returns:
            bool: True if successful, False otherwise
        """
        robot_name = robot_name or self.robot_name.lower()
        
        # Get the directory where this module is located
        current_dir = os.path.dirname(os.path.dirname(__file__))  # Go up one level from task_module
        context_file = os.path.join(current_dir, f'data/{robot_name}_context.txt')
        
        return self.load_context_from_file(context_file)
    
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
        return self.miscellaneous.set_leds("FaceLeds", red, green, blue, duration)
    
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
        return self.miscellaneous.set_leds(ear_name, red, green, blue, duration)
    
    def turn_off_leds(self, led_group: str = "AllLeds", duration: float = 0.0) -> bool:
        """
        Turn off a group of LEDs.
        
        Args:
            led_group (str): LED group name (default: "AllLeds")
            duration (float): Fade out duration in seconds
            
        Returns:
            bool: True if successful
        """
        return self.miscellaneous.set_leds(led_group, 0, 0, 0, duration)
    
    # Battery Monitoring Methods
    def get_battery_percentage(self) -> float | None:
        """
        Get the current battery percentage.
        
        Returns:
            float: Battery percentage (0-100), or None if not available
        """
        if self.miscellaneous.battery_percentage is not None:
            self.get_logger().info(f"Battery level: {self.miscellaneous.battery_percentage}%")
        else:
            self.get_logger().warn("Battery percentage not available yet")
        
        return self.miscellaneous.battery_percentage
