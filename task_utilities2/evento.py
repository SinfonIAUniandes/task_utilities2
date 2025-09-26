#!/usr/bin/env python3

# File: my_robot_py/my_robot_py/robot_interaction_node.py

import rclpy
from rclpy.node import Node
import sys

# Import the Speech class using the relative path from within the package.
# This assumes your 'speech_proxy.py' file is in a 'task_module' subdirectory.
from .task_module.speech_proxy import Speech


class RobotInteractionNode(Node):
    """
    An example node demonstrating how to use the Speech API from another file
    as part of a larger ROS2 application.
    """
    def __init__(self):
        super().__init__('robot_interaction_node')
        
        # 1. Instantiate the Speech API, passing this node object.
        # This allows the Speech class to use this node's clients and logger
        # without creating a separate, new node.
        self.get_logger().info("Initializing the Speech API...")
        try:
            self.speech = Speech(node=self)
            self.get_logger().info("Speech API initialized successfully. 🤖")
        except Exception as e:
            self.get_logger().fatal(f"Could not initialize Speech API: {e}")
            # Exit the application if essential components fail.
            sys.exit(1)
        
        # 2. Create a one-shot timer to run the main logic after initialization.
        # This is a robust pattern to ensure the node is fully ready before
        # starting the main application logic.

    def run_interaction_logic(self):
        """
        Contains the main logic for the robot's conversational interaction.
        """
        # Cancel the timer so this function only executes once.
        
        self.get_logger().info("Starting robot interaction sequence...")

        # --- Main Interaction Loop ---

        # a. The robot introduces itself and asks a question.
        intro_text = "Hello! I'm ready to help. What would you like to know?"
        self.get_logger().info(f"Robot says: '{intro_text}'")
        self.speech.say(text_say=intro_text,language_say="English",animated_say=True,asynchronous_say=False)

        # b. Listen for the user's response.
        self.get_logger().info("Listening for a question... 🎤")
        #question = self.speech.listen(autocut=False,timeout=int(5))
        question = "hola"

        # c. Process the transcribed text.
        if question and "ERROR" not in question:
            self.get_logger().info(f"Heard: '{question}'")
            
            # d. Formulate and speak a thinking message.
            thinking_message = "That's an interesting question. Let me think..."
            self.get_logger().info(f"Robot says: '{thinking_message}'")
            self.speech.say(thinking_message)
            
            # e. Get an answer from the Large Language Model.
            answer = self.speech.ask(question)
            
            # f. Speak the final answer.
            self.get_logger().info(f"Robot answers: '{answer}'")
            self.speech.say(answer)

        else:
            # Handle the case where listening failed or an error occurred.
            fail_message = "I'm sorry, I didn't quite catch that. Could you please try again?"
            self.get_logger().warn(fail_message)
            self.speech.say(fail_message)
        
        self.get_logger().info("Interaction sequence finished. Shutting down. 👋")
        
        # For this simple example, we shut down after one interaction.
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the node. The node's internal logic handles shutting down.
    interaction_node = RobotInteractionNode()
    interaction_node.run_interaction_logic()
    
    
    try:
        rclpy.spin(interaction_node)
    except KeyboardInterrupt:
        interaction_node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        # Ensure the node is destroyed on exit
        if rclpy.ok():
            interaction_node.destroy_node()
    


if __name__ == '__main__':
    main()