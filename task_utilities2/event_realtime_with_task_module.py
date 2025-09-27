#!/usr/bin/env python3

# Example: event_realtime_with_task_module.py
# This shows how to use TaskModule instead of Speech directly

import rclpy
from speech_msgs2.msg import Transcription
from threading import Thread
import os
import time

# Import the TaskModule class
from .task_module.task_module import TaskModule

# --- Global task_module object ---
rclpy.init()
task_module = TaskModule()

def process_transcription(transcription):
    """
    This function handles the blocking tasks (LLM and TTS calls)
    in a separate thread to avoid deadlocking the ROS 2 callback executor.
    It now also disables transcription during TTS to prevent a feedback loop.
    """
    print(f'Processing: "{transcription}"')
    
    # Using TaskModule.speech instead of direct Speech instance
    response = task_module.speech.ask(transcription)
    print(f'LLM Response: "{response}"')

    try:
        # --- FIX START ---
        # 1. Disable transcription to prevent the robot from hearing itself.
        print("Disabling transcription for TTS.")
        task_module.speech.set_transcription_mode(enabled=False)

        # 2. Make the robot speak. 
        task_module.speech.say(text_say=response, language_say="English", animated_say=True)
        time.sleep(2)
        # --- FIX END ---

    finally:
        # --- FIX START ---
        # 3. Re-enable transcription so the robot can listen for the user again.
        # This is in a 'finally' block to guarantee it runs even if speech.say() fails.
        print("Re-enabling transcription.")
        task_module.speech.set_transcription_mode(enabled=True, language='en')
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
    spin_thread = Thread(target=rclpy.spin, args=(task_module,))
    spin_thread.start()

    # Initialize for real-time interaction (loads context and enables transcription)
    # This replaces the manual context loading and transcription setup
    success = task_module.initialize_for_realtime_interaction(robot_name='nova', language='en')
    
    if not success:
        task_module.get_logger().error("Failed to initialize real-time interaction")
        return

    # Set up the subscription using the TaskModule's speech proxy
    task_module.speech.create_subscription(Transcription, '/microphone_node/transcription', on_transcription, 10)

    task_module.get_logger().info("Robot interaction node is running and waiting for transcription...")

    # The main thread can just wait. The spin_thread and processing_threads do the work.
    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        # Graceful shutdown
        task_module.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
