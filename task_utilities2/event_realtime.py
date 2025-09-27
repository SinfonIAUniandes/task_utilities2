#!/usr/bin/env python3

# File: my_robot_py/my_robot_py/robot_interaction_node.py

import rclpy
from rclpy.node import Node
from speech_msgs2.msg import Transcription
from threading import Thread
import os
import time

# Import the Speech class using the relative path from within the package.
# This assumes your 'speech_proxy.py' file is in a 'task_module' subdirectory.
from .task_module.speech_proxy2 import Speech

# --- Global speech object ---
# Initializing rclpy here is fine, but the object will be managed within main()
rclpy.init()
speech = Speech()

def process_transcription(transcription):
    """
    This function handles the blocking tasks (LLM and TTS calls)
    in a separate thread to avoid deadlocking the ROS 2 callback executor.
    It now also disables transcription during TTS to prevent a feedback loop.
    """
    print(f'Processing: "{transcription}"')
    response = speech.ask(transcription)
    print(f'LLM Response: "{response}"')

    try:
        # --- FIX START ---
        # 1. Disable transcription to prevent the robot from hearing itself.
        print("Disabling transcription for TTS.")
        speech.set_transcription_mode(enabled=False)

        # 2. Make the robot speak. The wait=True argument is important here.
        speech.say(text_say=response, language_say="English", animated_say=True, wait=True)
        time.sleep(2)
        # --- FIX END ---

    finally:
        # --- FIX START ---
        # 3. Re-enable transcription so the robot can listen for the user again.
        # This is in a 'finally' block to guarantee it runs even if speech.say() fails.
        print("Re-enabling transcription.")
        speech.set_transcription_mode(enabled=True, language='en')
        # --- FIX END ---

def on_transcription(msg):
    """
    This callback is now non-blocking. It simply receives the message
    and starts a new thread to handle the actual work.
    """
    print(f'I heard: "{msg.text}"')
    # Create and start a new thread to process the transcription
    # This prevents the callback itself from blocking.
    processing_thread = Thread(target=process_transcription, args=(msg.text,))
    processing_thread.start()

def main():
    # Run spin in a separate thread. This thread is dedicated to processing
    # incoming messages and executing their callbacks (like on_transcription).
    spin_thread = Thread(target=rclpy.spin, args=(speech,))
    spin_thread.start()

    # Load context for the LLM
    current_dir = os.path.dirname(__file__)
    nova_context_file = os.path.join(current_dir, 'data/nova_context.txt')
    try:
        with open(nova_context_file, 'r') as file:
            context = file.read()
            speech.set_llm_settings(context=context)
    except FileNotFoundError:
        speech.get_logger().error(f"Context file not found at {nova_context_file}")


    # Set up the subscription
    speech.set_transcription_mode(enabled=True, language='en')
    speech.create_subscription(Transcription, '/microphone_node/transcription', on_transcription, 10)

    speech.get_logger().info("Robot interaction node is running and waiting for transcription...")

    # The main thread can just wait. The spin_thread and processing_threads do the work.
    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        speech.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()