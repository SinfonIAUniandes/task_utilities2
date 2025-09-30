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
import os
import uuid
import operator
from typing import TypedDict, Annotated

from dotenv import load_dotenv
from langchain_core.messages import HumanMessage, ToolMessage
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_openai import AzureChatOpenAI
from langgraph.graph import StateGraph, END
from langgraph.prebuilt import ToolExecutor
from langgraph.checkpoint.sqlite import SqliteSaver

# Import TaskModule
from task_utilities2.task_module.task_module import TaskModule
from langchain.tools import tool

# Load environment variables from .env file
load_dotenv()


# Define the Agent's State
class AgentState(TypedDict):
    messages: Annotated[list, operator.add]


class RealtimeRobotAgent:
    """Real-time robot agent that responds to voice transcriptions."""

    def __init__(self, robot_name: str = "nova"):
        """Initialize the real-time robot agent."""
        self.robot_name = robot_name
        self.task_module = None
        self.app = None
        self.is_processing = False
        self.conversation_id = str(uuid.uuid4())

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

        # Create robot tools
        robot_tools = self._create_robot_tools()
        self.tool_executor = ToolExecutor(robot_tools)

        context = self.task_module.load_robot_context(robot_name=self.robot_name)

        # Initialize the model using Azure OpenAI
        llm = AzureChatOpenAI(
            temperature=0.3,
            azure_deployment=os.getenv("AZURE_OPENAI_DEPLOYMENT_NAME"),
            api_version=os.getenv("AZURE_OPENAI_API_VERSION"),
        )

        # Bind the tools to the model
        model_with_tools = llm.bind_tools(robot_tools)

        # --- Define the Prompt with Starting Context ---
        prompt = ChatPromptTemplate.from_messages(
            [
                (
                    "system",
                    context,
                ),
                MessagesPlaceholder(variable_name="messages"),
            ]
        )

        agent_runnable = prompt | model_with_tools

        # --- Define Graph Nodes ---
        self.agent_runnable = agent_runnable
        workflow = StateGraph(AgentState)
        workflow.add_node("agent", self.call_model)
        workflow.add_node("action", self.call_tool)
        workflow.set_entry_point("agent")
        workflow.add_conditional_edges(
            "agent",
            self.should_continue,
            {"continue": "action", "end": END},
        )
        workflow.add_edge("action", "agent")

        # --- Compile the Graph ---
        memory = SqliteSaver.from_conn_string(":memory:")
        self.app = workflow.compile(checkpointer=memory)

        # Initialize realtime transcription
        self.task_module.speech.set_transcription_mode(enabled=True, language='en')
        self.task_module.set_interactive_pepper_settings()

        print("Robot agent initialized! Available tools:")
        for tool_obj in robot_tools:
            print(f"  - {tool_obj.name}")
        return True

    # --- Node 1: The Agent ---
    def call_model(self, state: AgentState):
        """The 'brain' of the agent. Calls the LLM with the current message history."""
        print("--- Calling Model ---")
        response = self.agent_runnable.invoke(state)
        return {"messages": [response]}

    # --- Node 2: The Tool Executor ---
    def call_tool(self, state: AgentState):
        """Executes tools based on the agent's last message."""
        last_message = state["messages"][-1]

        tool_invocations = []
        for tool_call in last_message.tool_calls:
            action = (tool_call["name"], tool_call["args"])
            tool_invocations.append(action)

        print(f"--- Calling Tool(s): {', '.join([inv[0] for inv in tool_invocations])} ---")
        responses = self.tool_executor.batch(tool_invocations)

        tool_messages = [
            ToolMessage(content=str(response), tool_call_id=tool_call["id"])
            for tool_call, response in zip(last_message.tool_calls, responses)
        ]

        return {"messages": tool_messages}

    # --- Conditional Edge: should_continue ---
    def should_continue(self, state: AgentState):
        """Determines whether to continue with tool calls or end the conversation turn."""
        last_message = state["messages"][-1]
        if last_message.tool_calls:
            return "continue"
        else:
            # If there are no tool calls, we speak the response
            response_content = last_message.content
            if response_content:
                print(f"--- Speaking Response: {response_content} ---")
                self.task_module.speech.say(response_content, animated_say=True)
            return "end"

    def _create_robot_tools(self):
        """Create the essential robot tools for the agent."""

        @tool
        def robot_speak(text: str) -> str:
            """Make the robot speak out loud."""
            try:
                self.task_module.speech.say(text, animated_say=True)
                return f"Successfully said: '{text}'"
            except Exception as e:
                return f"Error speaking: {e}"

        @tool
        def change_eye_color(color_name: str, red: int = None, green: int = None, blue: int = None) -> str:
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
                elif red is not None and green is not None and blue is not None:
                    r, g, b = red, green, blue
                else:
                    return f"Unknown color '{color_name}'. Available colors: {', '.join(color_presets.keys())}."
                
                r, g, b = max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b))
                self.task_module.miscellaneous.set_eye_color(red=r, green=g, blue=b, duration=0.0)
                return f"Changed eye color to {color_name} (RGB: {r}, {g}, {b})"
            except Exception as e:
                return f"Error changing eye color: {e}"

        @tool
        def take_picture(filler: str = "") -> str:
            """Make the robot take a picture by playing the take picture animation."""
            try:
                success = self.task_module.miscellaneous.play_animation("Stand/Waiting/TakePicture_1")
                return "Successfully took a picture! *click* 📸" if success else "Could not take picture - animation failed"
            except Exception as e:
                return f"Error taking picture: {e}"

        @tool
        def get_current_time(query: str = "") -> str:
            """Get the current time."""
            from datetime import datetime
            current_time = datetime.now().strftime("%H:%M:%S")
            return f"Current time is {current_time}"

        return [robot_speak, change_eye_color, take_picture, get_current_time]

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
            config = {"configurable": {"thread_id": self.conversation_id}}
            inputs = {"messages": [HumanMessage(content=transcription_text)]}
            final_state = self.app.invoke(inputs, config=config)
            final_response = final_state["messages"][-1]

            print(f'Agent final response content: {final_response.content}')

            # Small delay to ensure any robot actions complete
            time.sleep(1)

        except Exception as e:
            print(f"Error processing transcription: {e}")
            try:
                self.task_module.speech.say("Sorry, I encountered an error processing your request.")
            except Exception as e2:
                print(f"Also failed to speak error message: {e2}")

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