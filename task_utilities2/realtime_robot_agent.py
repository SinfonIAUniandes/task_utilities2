#!/usr/bin/env python3

"""
Real-time Robot Agent

This agent listens to transcription messages from the microphone node and responds
using a ReAct LLM agent with the most relevant robot capabilities:
- Speaking (TTS)
- Changing eye colors 
- Taking pictures (playing animation)

The agent automatically manages transcription to prevent feedback loops and
provides natural conversational interaction.

Usage:
    python3 realtime_robot_agent.py
"""

import rclpy
from speech_msgs2.msg import Transcription
from threading import Thread
import time

# Import TaskModule and LLMAgent
from task_utilities2.task_module.task_module import TaskModule
from task_utilities2.task_module.logic.llm.llm_agent import LLMAgent
from langchain.tools import Tool


class RealtimeRobotAgent:
    """Real-time robot agent that responds to voice transcriptions."""
    
    def __init__(self, robot_name: str = "nova"):
        """Initialize the real-time robot agent."""
        self.robot_name = robot_name
        self.task_module = None
        self.robot_agent = None
        self.is_processing = False
        
    def initialize(self):
        """Initialize ROS2 and create the robot agent with tools."""
        # Initialize ROS2
        rclpy.init()
        
        # Create TaskModule instance
        self.task_module = TaskModule(
            node_name="realtime_robot_agent",
            robot_name=self.robot_name,
            enable_speech=True,
            enable_miscellaneous=True
        )
        
        # Start ROS2 spin in separate thread
        self.spin_thread = Thread(target=rclpy.spin, args=(self.task_module,))
        self.spin_thread.start()
        
        print(f"=== Real-time Robot Agent ({self.robot_name}) ===")
        print("Initializing robot agent with essential tools...")
        
        # Wait for services to be ready
        time.sleep(2)
        
        # Create robot tools - only the most relevant ones
        robot_tools = self._create_robot_tools()

        context = self.task_module.load_robot_context(robot_name=self.robot_name)

        # Initialize realtime transcription
        self.task_module.speech.set_transcription_mode(enabled=True, language='en')


        
        # Create the robot agent
        self.robot_agent = LLMAgent(
            initial_settings={
                "model_name": "gpt-4o-azure",
                "temperature": 0.3,
                "context": context
            },
            tools=robot_tools
        )

        self.task_module.set_interactive_pepper_settings()
        
        print("Robot agent initialized! Available tools:")
        for tool_name in self.robot_agent.list_tools():
            print(f"  - {tool_name}")
        return True
    
    def _create_robot_tools(self):
        """Create the essential robot tools for the agent."""
        
        def robot_speak(text: str) -> str:
            """Make the robot speak out loud."""
            try:
                self.task_module.speech.say(text, animated_say=True)
                return f"Successfully said: '{text}'"
            except Exception as e:
                return f"Error speaking: {e}"
        
        def change_eye_color(color_name: str, red: int = None, green: int = None, blue: int = None) -> str:
            """Change the robot's eye color. Can use color names or RGB values."""
            try:
                # Predefined colors for easy use
                color_presets = {
                    "red": (255, 0, 0),
                    "green": (0, 255, 0), 
                    "blue": (0, 0, 255),
                    "yellow": (255, 255, 0),
                    "purple": (128, 0, 128),
                    "cyan": (0, 255, 255),
                    "white": (255, 255, 255),
                    "orange": (255, 165, 0),
                    "pink": (255, 192, 203),
                    "lime": (50, 205, 50)
                }
                
                # Use preset color if available
                if color_name.lower() in color_presets:
                    r, g, b = color_presets[color_name.lower()]
                # Use provided RGB values
                elif red is not None and green is not None and blue is not None:
                    r, g, b = red, green, blue
                else:
                    return f"Unknown color '{color_name}'. Available colors: {', '.join(color_presets.keys())} or provide RGB values."
                
                # Ensure RGB values are in valid range
                r = max(0, min(255, r))
                g = max(0, min(255, g))
                b = max(0, min(255, b))
                
                self.task_module.set_eye_color(red=r, green=g, blue=b, duration=0.0)
                return f"Changed eye color to {color_name} (RGB: {r}, {g}, {b})"
                
            except Exception as e:
                return f"Error changing eye color: {e}"
        
        def take_picture(filler) -> str:
            """Make the robot take a picture by playing the take picture animation."""
            try:
                success = self.task_module.miscellaneous.play_animation("Stand/Waiting/TakePicture_1")
                if success:
                    return "Successfully took a picture! *click* 📸"
                else:
                    return "Could not take picture - animation failed"
            except Exception as e:
                return f"Error taking picture: {e}"
        
        def get_current_time(query: str = "") -> str:
            """Get the current time."""
            from datetime import datetime
            current_time = datetime.now().strftime("%H:%M:%S")
            return f"Current time is {current_time}"
        
        # Create the tool objects
        return [
            Tool(
                name="robot_speak",
                func=robot_speak,
                description="Make the robot speak text out loud with animation. Use this to communicate verbally with the user."
            ),
            Tool(
                name="change_eye_color",
                func=change_eye_color,
                description="Change the robot's eye color for visual feedback. Input: color_name (red, green, blue, yellow, purple, cyan, white, orange, pink, lime) or specify red, green, blue RGB values (0-255)."
            ),
            Tool(
                name="take_picture",
                func=take_picture,
                description="Make the robot take a picture by playing a camera animation. No input needed."
            ),
            Tool(
                name="get_current_time",
                func=get_current_time,
                description="Get the current time. No input needed."
            )
        ]
    
    def process_transcription(self, transcription_text: str):
        """Process transcription in a separate thread to avoid blocking."""
        if self.is_processing:
            print("Already processing a request, ignoring new transcription...")
            return
            
        self.is_processing = True
        
        try:
            print(f'User said: "{transcription_text}"')
            
            # Disable transcription to prevent feedback loop
            print("Disabling transcription during response...")
            self.task_module.speech.set_transcription_mode(enabled=False)
            
            # Get response from the LLM agent
            response = self.robot_agent.get_response(transcription_text)
            print(f'Agent response: {response}')
            
            # Small delay to ensure any robot actions complete
            time.sleep(1)
            
        except Exception as e:
            print(f"Error processing transcription: {e}")
            # Still try to speak an error message
            try:
                self.task_module.speech.say("Sorry, I encountered an error processing your request.")
            except:
                pass
        
        finally:
            # Always re-enable transcription
            print("Re-enabling transcription...")
            time.sleep(1)  # Brief pause before re-enabling
            self.task_module.speech.set_transcription_mode(enabled=True, language='en')
            self.is_processing = False
    
    def on_transcription(self, msg):
        """Callback for transcription messages - non-blocking."""
        if not msg.text.strip():  # Ignore empty transcriptions
            return
            
        # Start processing in a separate thread
        processing_thread = Thread(
            target=self.process_transcription, 
            args=(msg.text,)
        )
        processing_thread.start()
    
    def start_listening(self):
        """Start listening for transcription messages."""
        # Set up subscription to transcription topic
        self.task_module.speech.create_subscription(
            Transcription, 
            '/microphone_node/transcription', 
            self.on_transcription, 
            10
        )
        
        print("🎤 Real-time robot agent is now listening for voice input...")
        print("Say something to interact with the robot!")
        print("Available capabilities:")
        print("  • Natural conversation")
        print("  • Voice responses")
        print("  • Eye color changes")
        print("  • Taking pictures")
        print("  • Time queries")
        print("\nPress Ctrl+C to stop")
        
        # Keep the main thread alive
        try:
            while rclpy.ok():
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down real-time robot agent...")
    
    def shutdown(self):
        """Clean shutdown of the agent."""
        if self.task_module:
            self.task_module.destroy_node()
        rclpy.shutdown()
        if hasattr(self, 'spin_thread'):
            self.spin_thread.join()


def main():
    """Main function to run the real-time robot agent."""
    agent = RealtimeRobotAgent(robot_name="nova")
    
    try:
        # Initialize the agent
        if not agent.initialize():
            print("Failed to initialize robot agent!")
            return
        
        # Start listening for voice input
        agent.start_listening()
        
    except Exception as e:
        print(f"Agent failed with error: {e}")
    
    finally:
        agent.shutdown()


if __name__ == '__main__':
    main()