#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from threading import Thread
import os

# Import proxy classes
from .speech_proxy import Speech
from .navigation_proxy import NavigationProxy


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
                 robot_name=None):
        """
        Initialize the TaskModule with all proxy systems.
        
        Args:
            node_name (str): Name for this ROS2 node
            microphone_node_name (str): Name of the microphone node for speech services
            conversation_node_name (str): Name of the conversation node for LLM services
            robot_name (str): Name of the robot (optional)
        """
        super().__init__(node_name)
        
        self.robot_name = robot_name or "robot"
        self.microphone_node_name = microphone_node_name
        self.conversation_node_name = conversation_node_name
        
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
    
    def initialize_for_realtime_interaction(self, robot_name=None, language='en'):
        """
        Initialize the task module for real-time interaction scenarios.
        
        This method sets up the robot for continuous conversation by:
        1. Loading the appropriate context
        2. Enabling transcription mode
        
        Args:
            robot_name (str): Name of the robot for context loading
            language (str): Language for transcription
            
        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info("Initializing for real-time interaction...")
        
        # Load robot context
        if not self.load_robot_context(robot_name):
            self.get_logger().warn("Could not load robot context, continuing without it")
        
        # Enable transcription mode
        success = self.speech.set_transcription_mode(enabled=True, language=language)
        
        if success:
            self.get_logger().info("Real-time interaction mode initialized successfully")
        else:
            self.get_logger().error("Failed to initialize real-time interaction mode")
            
        return success
    
    def shutdown_gracefully(self):
        """Gracefully shutdown all proxy systems."""
        self.get_logger().info("Shutting down TaskModule...")
        
        # Shutdown speech proxy if it has cleanup methods
        if hasattr(self.speech, 'shutdown'):
            self.speech.shutdown()
        
        # Add shutdown for other proxies when implemented
        
        self.get_logger().info("TaskModule shutdown complete")