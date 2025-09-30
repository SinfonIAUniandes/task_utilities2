#!/usr/bin/env python3
from rclpy.node import Node
import os
import time

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
        task_module = TaskModule(speech=True, miscellaneous=True)
        task_module.speech.say("Hello World")
        response = task_module.speech.ask("What is your name?")
    """
    
    def __init__(self, 
                 node_name="task_module",
                 microphone_node_name="microphone_node",
                 conversation_node_name="conversation_node",
                 manipulation_node_name="naoqi_manipulation_node",
                 robot_name=None,
                 enable_speech=False,
                 enable_navigation=False,
                 enable_miscellaneous=False):
        """
        Initialize the TaskModule with selected proxy systems.
        
        Args:
            node_name (str): Name for this ROS2 node
            microphone_node_name (str): Name of the microphone node for speech services
            conversation_node_name (str): Name of the conversation node for LLM services
            manipulation_node_name (str): Name of the manipulation node for robot control
            robot_name (str): Name of the robot (optional)
            enable_speech (bool): Whether to initialize the speech proxy (default: False)
            enable_navigation (bool): Whether to initialize the navigation proxy (default: False)
            enable_miscellaneous (bool): Whether to initialize the miscellaneous proxy (default: False)
        """
        super().__init__(node_name)
        
        self.robot_name = robot_name or "robot"
        self.microphone_node_name = microphone_node_name
        self.conversation_node_name = conversation_node_name
        self.manipulation_node_name = manipulation_node_name
        
        # Store proxy enable flags
        self.enable_speech = enable_speech
        self.enable_navigation = enable_navigation
        self.enable_miscellaneous = enable_miscellaneous
        
        # Initialize proxy attributes as None
        self.speech = None
        self.navigation = None
        self.miscellaneous = None
        
        self.get_logger().info(f"Initializing TaskModule for robot: {self.robot_name}")
        self.get_logger().info(f"Proxy initialization flags - Speech: {enable_speech}, Navigation: {enable_navigation}, Miscellaneous: {enable_miscellaneous}")
        
        # Initialize proxy systems conditionally
        self._initialize_proxies()
        
        self.get_logger().info("TaskModule initialization complete")
    
    def _initialize_proxies(self):
        """Initialize enabled proxy systems."""
        self.get_logger().info("Initializing enabled proxy systems...")
        
        if self.enable_speech:
            self._initialize_speech_proxy()
        else:
            self.get_logger().info("Speech proxy disabled - skipping initialization")
        
        if self.enable_navigation:
            self._initialize_navigation_proxy()
        else:
            self.get_logger().info("Navigation proxy disabled - skipping initialization")
        
        if self.enable_miscellaneous:
            self._initialize_miscellaneous_proxy()
        else:
            self.get_logger().info("Miscellaneous proxy disabled - skipping initialization")
        
        # Add more proxies here in the future
        # if self.enable_vision:
        #     self._initialize_vision_proxy()
        # if self.enable_manipulation:
        #     self._initialize_manipulation_proxy()
        
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

    def _check_proxy_availability(self, proxy_name: str) -> bool:
        """
        Check if a proxy is available and initialized.
        
        Args:
            proxy_name (str): Name of the proxy to check
            
        Returns:
            bool: True if proxy is available, False otherwise
        """
        proxy = getattr(self, proxy_name, None)
        if proxy is None:
            self.get_logger().warn(f"{proxy_name.capitalize()} proxy not initialized. Enable it during TaskModule creation.")
            return False
        return True
    
    def set_default_pepper_settings(self):
        if self._check_proxy_availability('miscellaneous'):
            self.miscellaneous.set_autonomous_state(False)
            time.sleep(1)
            self.miscellaneous.toggle_awareness(False)
            time.sleep(1)
            self.miscellaneous.go_to_posture("Stand")
            time.sleep(1)
            self.miscellaneous.toggle_breathing("All", False)
        if self._check_proxy_availability('speech'):
            self.speech.set_volume(70)
    
    def set_interactive_pepper_settings(self):
        if self._check_proxy_availability('miscellaneous'):
            self.miscellaneous.set_autonomous_state(False)
            time.sleep(1)
            self.miscellaneous.toggle_awareness(True)
            time.sleep(1)
            self.miscellaneous.go_to_posture("Stand")
            time.sleep(1)
            self.miscellaneous.toggle_breathing("All", True)
        if self._check_proxy_availability('speech'):
            self.speech.set_volume(70)

    def load_robot_context(self, robot_name=None,language=None):
        """
        Load context for a specific robot from the data directory.
        
        Args:
            robot_name (str): Name of the robot (nova, opera, orion). 
                            If None, uses self.robot_name
            
        Returns:
            str: Robot context as string, or False if failed
        """
        robot_name = robot_name or self.robot_name.lower()
        
        # Get the directory where this module is located
        current_dir = os.path.dirname(os.path.dirname(__file__)).replace("build","src")  # Go up one level from task_module
        context_file_path = os.path.join(current_dir, f'data/{robot_name}_context.txt')
        if language:
            context_file_path = os.path.join(current_dir, f'data/{language}/{robot_name}_context.txt')


        general_SinfonIA_context_path = os.path.join(current_dir, 'data/info_herramientas_SinfonIA.txt')
        if language:
            general_SinfonIA_context_path = os.path.join(current_dir, f'data/{language}/info_herramientas_SinfonIA.txt')
        
        try:
            with open(general_SinfonIA_context_path,"r",encoding="utf-8") as general_context_file:
                general_context = general_context_file.read()
                with open(context_file_path, 'r') as file:
                    full_robot_context = file.read().replace("{info_herramientas}",general_context)
                    return full_robot_context
        except FileNotFoundError:
            self.get_logger().error(f"Context file not found at {context_file_path}")
            return False
        except Exception as e:
            self.get_logger().error(f"Error loading context file: {e}")
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
        if not self._check_proxy_availability('miscellaneous'):
            return False
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
        if not self._check_proxy_availability('miscellaneous'):
            return False
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
        if not self._check_proxy_availability('miscellaneous'):
            return False
        return self.miscellaneous.set_leds(led_group, 0, 0, 0, duration)
    
    # Battery Monitoring Methods
    def get_battery_percentage(self) -> float | None:
        """
        Get the current battery percentage.
        
        Returns:
            float: Battery percentage (0-100), or None if not available
        """
        if not self._check_proxy_availability('miscellaneous'):
            return None
            
        if self.miscellaneous.battery_percentage is not None:
            self.get_logger().info(f"Battery level: {self.miscellaneous.battery_percentage}%")
        else:
            self.get_logger().warn("Battery percentage not available yet")
        
        return self.miscellaneous.battery_percentage
