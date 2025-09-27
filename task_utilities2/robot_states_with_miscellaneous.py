#!/usr/bin/env python3

# Example: robot_states_with_miscellaneous.py
# This shows how to use MiscellaneousProxy for different robot states

import rclpy
from threading import Thread
import time

from .task_module.task_module import TaskModule

class RobotStateManager:
    """
    Example class showing how to use MiscellaneousProxy to manage robot states.
    This demonstrates practical usage patterns for the miscellaneous functionality.
    """
    
    def __init__(self):
        rclpy.init()
        
        self.task_module = TaskModule(robot_name="nova")
        
        # Start ROS2 spinning in background
        self.spin_thread = Thread(target=rclpy.spin, args=(self.task_module,))
        self.spin_thread.start()
        
        # Wait for services to be ready
        time.sleep(2)
    
    def initialize_robot(self):
        """Initialize robot to default operational state."""
        print("Initializing robot to operational state...")
        
        # Set up default miscellaneous state
        self.task_module.miscellaneous.setup_default_state()
        
        # Announce readiness
        self.task_module.miscellaneous.set_eyes_green(duration=1.0)
        self.task_module.speech.say("System initialized. I am ready for operation.", animated_say=True)
        
        time.sleep(2)
    
    def enter_sleep_mode(self):
        """Put robot in sleep/standby mode."""
        print("Entering sleep mode...")
        
        # Announce sleep mode
        self.task_module.speech.say("Entering sleep mode. Good night!", animated_say=True)
        time.sleep(2)
        
        # Disable autonomous behaviors
        self.task_module.miscellaneous.disable_awareness()
        self.task_module.miscellaneous.disable_autonomous_life()
        self.task_module.miscellaneous.disable_blinking()
        
        # Dim the lights
        self.task_module.miscellaneous.set_eye_color(red=20, green=20, blue=40, duration=2.0)
        
        print("Robot is now in sleep mode")
    
    def wake_up(self):
        """Wake robot from sleep mode."""
        print("Waking up robot...")
        
        # Gradually brighten eyes
        self.task_module.miscellaneous.set_eyes_white(duration=2.0)
        time.sleep(1)
        
        # Re-enable autonomous behaviors
        self.task_module.miscellaneous.enable_blinking()
        self.task_module.miscellaneous.enable_awareness()
        self.task_module.miscellaneous.enable_autonomous_life()
        
        # Announce wake up
        self.task_module.speech.say("Good morning! I'm awake and ready to help.", animated_say=True)
        
        print("Robot is now awake")
        time.sleep(2)
    
    def enter_presentation_mode(self):
        """Configure robot for presentation/demo mode."""
        print("Entering presentation mode...")
        
        # Disable autonomous life to prevent unexpected movements
        self.task_module.miscellaneous.disable_autonomous_life()
        
        # Enable awareness to track audience
        self.task_module.miscellaneous.enable_awareness()
        
        # Set professional appearance
        self.task_module.miscellaneous.set_eyes_white(duration=1.0)
        self.task_module.miscellaneous.enable_blinking()
        
        self.task_module.speech.say("I am now in presentation mode, ready for demonstration.", animated_say=True)
        
        print("Robot is in presentation mode")
        time.sleep(2)
    
    def enter_interactive_mode(self):
        """Configure robot for interactive conversation."""
        print("Entering interactive mode...")
        
        # Enable all autonomous behaviors for natural interaction
        self.task_module.miscellaneous.enable_autonomous_life()
        self.task_module.miscellaneous.enable_awareness()
        self.task_module.miscellaneous.enable_blinking()
        
        # Set friendly eye color
        self.task_module.miscellaneous.set_eyes_cyan(duration=1.0)
        
        self.task_module.speech.say("I'm now in interactive mode. Feel free to talk with me!", animated_say=True)
        
        print("Robot is in interactive mode")
        time.sleep(2)
    
    def handle_low_battery(self):
        """Handle low battery situation."""
        battery_level = self.task_module.miscellaneous.get_battery_percentage()
        
        if self.task_module.miscellaneous.is_battery_critical():
            print("CRITICAL: Battery is critically low!")
            
            # Visual warning with red eyes
            self.task_module.miscellaneous.set_eyes_red(duration=0.5)
            
            # Announce critical battery
            self.task_module.speech.say("Warning! My battery is critically low. I need to be charged immediately.", animated_say=True)
            
            # Enter power saving mode
            self.enter_power_saving_mode()
            
            return True
            
        elif self.task_module.miscellaneous.is_battery_low():
            print("WARNING: Battery is low")
            
            # Visual warning with yellow eyes
            self.task_module.miscellaneous.set_eyes_yellow(duration=1.0)
            
            # Announce low battery
            self.task_module.speech.say("My battery is getting low. Please consider charging me soon.", animated_say=True)
            
            return True
        
        return False
    
    def enter_power_saving_mode(self):
        """Enter power saving mode to conserve battery."""
        print("Entering power saving mode...")
        
        # Disable power-consuming autonomous behaviors
        self.task_module.miscellaneous.disable_autonomous_life()
        self.task_module.miscellaneous.disable_awareness()
        
        # Dim all LEDs to save power
        self.task_module.miscellaneous.set_eye_color(red=50, green=50, blue=0, duration=1.0)  # Dim yellow
        
        self.task_module.speech.say("Entering power saving mode to conserve battery.", animated_say=True)
        
        print("Robot is in power saving mode")
        time.sleep(2)
    
    def express_emotion(self, emotion: str):
        """Express different emotions through LEDs and speech."""
        emotions = {
            "happy": {
                "color": (255, 255, 0),  # Yellow
                "message": "I'm feeling happy!",
                "duration": 1.0
            },
            "sad": {
                "color": (0, 0, 255),  # Blue
                "message": "I'm feeling a bit sad.",
                "duration": 2.0
            },
            "excited": {
                "color": (255, 0, 255),  # Magenta
                "message": "I'm so excited!",
                "duration": 0.5
            },
            "calm": {
                "color": (0, 255, 0),  # Green
                "message": "I'm feeling calm and peaceful.",
                "duration": 2.0
            },
            "curious": {
                "color": (0, 255, 255),  # Cyan
                "message": "I'm curious about something.",
                "duration": 1.0
            },
            "focused": {
                "color": (255, 255, 255),  # White
                "message": "I'm focusing on the task.",
                "duration": 1.0
            }
        }
        
        if emotion in emotions:
            emotion_data = emotions[emotion]
            r, g, b = emotion_data["color"]
            
            print(f"Expressing emotion: {emotion}")
            
            self.task_module.miscellaneous.set_eye_color(r, g, b, duration=emotion_data["duration"])
            self.task_module.speech.say(emotion_data["message"], animated_say=True)
            
            time.sleep(2)
        else:
            print(f"Unknown emotion: {emotion}")
    
    def shutdown(self):
        """Gracefully shutdown the robot state manager."""
        print("Shutting down robot state manager...")
        
        # Return to neutral state
        self.task_module.miscellaneous.set_eyes_white(duration=1.0)
        self.task_module.speech.say("Shutting down. Goodbye!", animated_say=True)
        
        time.sleep(3)
        
        # Shutdown TaskModule
        self.task_module.shutdown_gracefully()
        self.task_module.destroy_node()
        rclpy.shutdown()
        self.spin_thread.join()

def main():
    """Main function demonstrating robot state management."""
    state_manager = RobotStateManager()
    
    try:
        # Demonstration sequence
        print("=== Robot State Management Demo ===")
        
        # 1. Initialize robot
        state_manager.initialize_robot()
        
        # 2. Check battery status
        if not state_manager.handle_low_battery():
            print("Battery level is adequate")
        
        # 3. Demo different operational modes
        state_manager.enter_interactive_mode()
        time.sleep(3)
        
        state_manager.enter_presentation_mode()
        time.sleep(3)
        
        # 4. Demo emotions
        emotions = ["happy", "excited", "curious", "calm", "focused"]
        for emotion in emotions:
            state_manager.express_emotion(emotion)
        
        # 5. Demo sleep/wake cycle
        state_manager.enter_sleep_mode()
        time.sleep(5)  # Simulate sleep period
        
        state_manager.wake_up()
        time.sleep(3)
        
        # 6. Final check
        state_manager.handle_low_battery()
        
        print("Demo completed successfully!")
        
    except KeyboardInterrupt:
        print("Demo interrupted by user")
    finally:
        state_manager.shutdown()

if __name__ == '__main__':
    main()
