#!/usr/bin/env python3

# Example: miscellaneous_functionality_example.py
# This shows how to use the MiscellaneousProxy through TaskModule

import rclpy
from threading import Thread
import time

# Import the TaskModule class
from task_utilities2.task_module.task_module import TaskModule

def main():
    rclpy.init()
    
    # Create TaskModule instance
    task_module = TaskModule(robot_name="nova")
    
    # Run spin in separate thread
    spin_thread = Thread(target=rclpy.spin, args=(task_module,))
    spin_thread.start()
    
    print("=== TaskModule Miscellaneous Functionality Demo ===")
    
    # Wait a moment for services to be ready
    time.sleep(2)
    
    # 1. Battery monitoring
    print("\n1. Battery Monitoring:")
    battery_level = task_module.get_battery_percentage()
    if battery_level is not None:
        print(f"   Current battery level: {battery_level}%")
    else:
        print("   Battery level not available yet")
    
    # 2. LED Control Demo
    print("\n2. LED Control Demo:")
    print("   Setting eyes to different colors...")
    
    # Custom color (purple)
    task_module.set_eye_color(red=128, green=0, blue=128, duration=1.0)
    task_module.speech.say("This is a custom purple color", animated_say=True)
    time.sleep(2)
    
    # 3. Autonomous Life Demo
    print("\n3. Autonomous Life Control:")
    
    # Disable autonomous life for demonstration
    print("   Disabling autonomous life...")
    if task_module.miscellaneous.set_autonomous_state(False):
        task_module.speech.say("Autonomous life disabled", animated_say=True)
        time.sleep(2)
    
    # Re-enable it
    print("   Re-enabling autonomous life...")
    if task_module.miscellaneous.set_autonomous_state(True):
        task_module.speech.say("Autonomous life enabled", animated_say=True)
        time.sleep(2)
    
    # 4. Basic Awareness Demo
    print("\n4. Basic Awareness Control:")
    
    # Disable awareness
    print("   Disabling basic awareness...")
    if task_module.miscellaneous.toggle_awareness(False):
        task_module.speech.say("I will not move my head to track you now", animated_say=True)
        time.sleep(3)
    
    # Re-enable awareness
    print("   Re-enabling basic awareness...")
    if task_module.miscellaneous.toggle_awareness(True):
        task_module.speech.say("Now I can track you with my head again", animated_say=True)
        time.sleep(2)
    
    # 5. Blinking Control Demo
    print("\n5. Blinking Control:")
    
    # Disable blinking
    print("   Disabling autonomous blinking...")
    if task_module.miscellaneous.toggle_blinking(False):
        task_module.speech.say("I will stop blinking now", animated_say=True)
        time.sleep(3)
    
    # Re-enable blinking
    print("   Re-enabling autonomous blinking...")
    if task_module.miscellaneous.toggle_blinking(True):
        task_module.speech.say("Now my eyes will blink naturally again", animated_say=True)
        time.sleep(2)
    
    # 6. LED Effects Demo
    print("\n6. LED Effects Demo:")
    
    # Ear color control
    print("   Setting ear colors...")
    task_module.set_ear_color("Left", red=255, green=0, blue=0, duration=1.0)  # Red left ear
    task_module.set_ear_color("Right", red=0, green=0, blue=255, duration=1.0)  # Blue right ear
    task_module.speech.say("My left ear is red and my right ear is blue", animated_say=True)
    time.sleep(3)
    
    # Color cycle
    print("   Color cycling demo...")
    colors = [
        ("red", 255, 0, 0),
        ("green", 0, 255, 0),
        ("blue", 0, 0, 255),
        ("yellow", 255, 255, 0),
        ("purple", 128, 0, 128),
        ("cyan", 0, 255, 255)
    ]
    
    for color_name, r, g, b in colors:
        task_module.set_eye_color(r, g, b, duration=0.5)
        time.sleep(0.8)
    
    task_module.speech.say("That concludes the miscellaneous functionality demonstration. Thank you!", animated_say=True)
    
    # Wait for speech to complete
    time.sleep(5)
    
    # Cleanup
    print("\nShutting down...")
    task_module.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
