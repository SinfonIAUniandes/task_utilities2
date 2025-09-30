from langchain_openai import AzureChatOpenAI
from langgraph.prebuilt import create_react_agent
from langchain_core.tools import tool
from dotenv import load_dotenv
import os
import rclpy
from speech_msgs2.msg import Transcription
from threading import Thread
from task_utilities2.task_module.task_module import TaskModule
from langchain_core.messages import HumanMessage
import time
import random

load_dotenv()

# Initialize ROS2
rclpy.init()

# Create TaskModule instance
task_module = TaskModule(
    node_name="realtime_robot_agent",
    robot_name="nova",
    enable_speech=True,
    enable_miscellaneous=True
)

@tool
def robot_speak(text: str) -> str:
    """Make the robot speak out loud."""
    try:
        task_module.speech.say(text, animated_say=True)
        return f"Successfully said: '{text}'"
    except Exception as e:
        return f"Error speaking: {e}"

@tool
def change_eye_color(color_name: str) -> str:
    """Change the robot's eye color. Can use color names or RGB values."""
    try:
        color_presets = {
            "red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255),
            "yellow": (255, 255, 0), "purple": (128, 0, 128), "cyan": (0, 255, 255),
            "white": (255, 255, 255), "orange": (255, 165, 0), "pink": (255, 192, 203),
            "lime": (50, 205, 50)
        }
        if color_name.lower() in color_presets:
            r, g, b = color_presets[color_name.lower()]
        else:
            return f"Unknown color '{color_name}'. Available colors: {', '.join(color_presets.keys())}."
        
        r, g, b = max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b))
        task_module.miscellaneous.set_eye_color(red=r, green=g, blue=b, duration=0.0)
        return f"Changed eye color to {color_name} (RGB: {r}, {g}, {b})"
    except Exception as e:
        return f"Error changing eye color: {e}"

@tool
def take_picture() -> str:
    """Makes the robot take a picture by playing the take picture animation."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/TakePicture_1")
        return "Successfully took a picture!"
    except Exception as e:
        return f"Error taking picture: {e}"

@tool
def get_current_time() -> str:
    """Get the current time."""
    from datetime import datetime
    current_time = datetime.now().strftime("%H:%M:%S")
    return f"Current time is {current_time}"

@tool
def play_dance() -> str:
    """Makes the robot do a random dance"""
    try:
        dances = ["Stand/arcadia/full_launcher","Stand/asereje/full_launcher","Stand/jgangnamstyle/full_launcher","Stand/Macarena/full_launcher","Stand/jammin/full_launcher","Stand/disco/full_launcher"]
        chosen_dance = random.choice(dances)
        task_module.miscellaneous.play_animation(chosen_dance)
        return "Successfully danced!"
    except Exception as e:
        return f"Error taking picture: {e}"
    
@tool
def play_guitar() -> str:
    """Makes the robot play the guitar"""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/AirGuitar_1")
        return "Successfully played the guitar!"
    except Exception as e:
        return f"Error taking picture: {e}"
    
@tool
def throw_kisses() -> str:
    """Makes the robot throw kisses"""
    try:
        task_module.miscellaneous.play_animation("Stand/Gestures/Kisses_1")
        return "Successfully threw kisses!"
    except Exception as e:
        return f"Error taking picture: {e}"
    
@tool
def pose_for_picture() -> str:
    """Makes the robot strike a pose for a picture"""
    try:
        task_module.miscellaneous.toggle_breathing(joint_group="All",enabled=False)
        task_module.miscellaneous.play_animation("Stand/Gestures/ShowSky_8")
        time.sleep(2)
        task_module.miscellaneous.toggle_breathing(joint_group="All",enabled=True)
        return "Successfully posed!"
    except Exception as e:
        return f"Error taking picture: {e}"
    
@tool
def show_heart_gesture() -> str:
    """Makes the robot show a 'Love You/Heart' gesture."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/LoveYou_1")
        return "Successfully showed a heart gesture!"
    except Exception as e:
        return f"Error showing heart gesture: {e}"

@tool
def make_a_call_gesture() -> str:
    """Makes the robot play the 'Call Someone' animation."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/CallSomeone_1")
        return "Successfully played the 'Call Someone' animation!"
    except Exception as e:
        return f"Error playing 'Call Someone': {e}"

@tool
def act_like_a_zombie() -> str:
    """Makes the robot play the 'Zombie' animation."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/Zombie_1")
        return "Successfully played the 'Zombie' animation!"
    except Exception as e:
        return f"Error playing 'Zombie': {e}"

@tool
def drive_a_car_gesture() -> str:
    """Makes the robot play the 'Drive Car' animation."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/DriveCar_1")
        return "Successfully played the 'Drive Car' animation!"
    except Exception as e:
        return f"Error playing 'Drive Car': {e}"

@tool
def show_muscles() -> str:
    """Makes the robot play the 'Show Muscles' animation."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/ShowMuscles_3")
        return "Successfully played the 'Show Muscles' animation!"
    except Exception as e:
        return f"Error playing 'Show Muscles': {e}"


@tool
def perform_bow_gesture() -> str:
    """Makes the robot play the 'Bow' gesture (e.g., as thanks)."""
    try:
        task_module.miscellaneous.play_animation("Stand/Gestures/BowShort_3")
        return "Successfully performed the bow gesture!"
    except Exception as e:
        return f"Error performing bow: {e}"

tools = [robot_speak, change_eye_color, take_picture, get_current_time,play_dance,show_heart_gesture,make_a_call_gesture,act_like_a_zombie,drive_a_car_gesture,show_muscles,perform_bow_gesture]


class RealtimeRobotAgent:
    """Real-time robot agent that responds to voice transcriptions."""

    def __init__(self, robot_name: str = "nova"):
        """Initialize the real-time robot agent."""
        self.robot_name = robot_name
        self.robot_agent = None
        self.is_processing = False

    def initialize(self):
        """Initialize ROS2 and create the robot agent with tools."""
        
        # Start ROS2 spin in separate thread
        self.spin_thread = Thread(target=rclpy.spin, args=(task_module,))
        self.spin_thread.start()
        
        print(f"=== Real-time Robot Agent ({self.robot_name}) ===")
        print("Initializing robot agent with essential tools...")
        
        # Wait for services to be ready
        time.sleep(2)

        context = task_module.load_robot_context(robot_name=self.robot_name) + "Response in a nice and simple manner and do NOT return weird formatting like markdown."

        self.llm = AzureChatOpenAI(
            azure_deployment=os.getenv("AZURE_OPENAI_CHAT_DEPLOYMENT"),
            openai_api_version=os.getenv("AZURE_OPENAI_API_VERSION"),
            api_key=os.getenv("AZURE_OPENAI_API_KEY"),
            azure_endpoint=os.getenv("AZURE_OPENAI_ENDPOINT"),
        )

        self.robot_agent = create_react_agent(
            model = self.llm,
            tools=tools,
            prompt=context
        )

        # Initialize realtime transcription
        task_module.speech.set_transcription_mode(enabled=True, language='en')

        task_module.set_interactive_pepper_settings()
    
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
            task_module.speech.set_transcription_mode(enabled=False)
            
            agent_input = {
                "messages": [HumanMessage(content=transcription_text)]
            }
            # Get response from the LLM agent
            response = self.robot_agent.invoke(agent_input) 
            
            # 3. Extract the final response text from the agent's output
            final_message = response['messages'][-1]
            final_response_text = final_message.content
            
            print(f'Agent final message: {final_response_text}')
            
            # Use the extracted final text for robot_speak (if the agent didn't already use the tool)
            if not final_message.tool_calls:
                # If the agent's final message is a text response (not a tool call), have the robot speak it.
                # If the agent called a tool (like robot_speak), the tool execution already happened.
                # In a standard ReAct setup, the final AI message is the spoken response.
                task_module.speech.say(final_response_text, animated_say=True)
                
            # If the final message *was* a tool call, the tool execution would have occurred 
            # within the agent's loop, and the `robot_speak` tool would have handled the output.
            
            # Small delay to ensure any robot actions complete
            time.sleep(1)
            
        except Exception as e:
            print(f"Error processing transcription: {e}")
            # Still try to speak an error message
            try:
                task_module.speech.say("Sorry, I encountered an error processing your request.")
            except:
                pass
        
        finally:
            # Always re-enable transcription
            print("Re-enabling transcription...")
            time.sleep(1)  # Brief pause before re-enabling
            task_module.speech.set_transcription_mode(enabled=True, language='en')
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
        task_module.speech.create_subscription(
            Transcription, 
            '/microphone_node/transcription', 
            self.on_transcription, 
            10
        )
        
        print("🎤 Real-time robot agent is now listening for voice input...")
        print("Say something to interact with the robot!")
        
        # Keep the main thread alive
        try:
            while rclpy.ok():
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down real-time robot agent...")
    
    def shutdown(self):
        """Clean shutdown of the agent."""
        if task_module:
            task_module.destroy_node()
        rclpy.shutdown()
        if hasattr(self, 'spin_thread'):
            self.spin_thread.join()


def main():
    """Main function to run the real-time robot agent."""
    agent = RealtimeRobotAgent(robot_name="nova")
    
    try:
        # Initialize the agent
        agent.initialize()
        # Start listening for voice input
        agent.start_listening()
        
    except Exception as e:
        print(f"Agent failed with error: {e}")
    
    finally:
        agent.shutdown()


if __name__ == '__main__':
    main()



