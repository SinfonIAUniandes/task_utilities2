#!/usr/bin/env python3

# File: my_robot_py/my_robot_py/robot_interaction_node.py

import rclpy
from threading import Thread

# Import the Speech class using the relative path from within the package.
# This assumes your 'speech_proxy.py' file is in a 'task_module' subdirectory.
from .task_module.speech_proxy2 import Speech

def main():
    rclpy.init()

    speech = Speech()

    # Run spin in a separate thread so .call() works synchronously
    spin_thread = Thread(target=rclpy.spin, args=(speech,))
    spin_thread.start()

    speech.say("Hello, this is a test",wait=True)
    heard = speech.listen(autocut=False,timeout=2)
    speech.say(heard,wait=True)
    llm_response = speech.ask(heard)
    speech.say(llm_response,wait=True)


    speech.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()