#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Note: This script assumes that the 'speech_msgs2' and 'std_srvs' packages,
# which define the service types, are available in your ROS2 workspace.
from speech_msgs2.srv import SpeechToText, RecordAudio, SetTranscriptionMode
from speech_msgs2.srv import SetLLMSettings, LLMResponse
from naoqi_utilities_msgs.srv import Say
from std_srvs.srv import Trigger


class Speech(Node):
    """
    A Python class to interact with speech and conversation ROS2 nodes.
    """

    def __init__(self,
                 microphone_node_name="microphone_node",
                 conversation_node_name="conversation_node"):
        """
        Initializes the Speech API object.

        Args:
            microphone_node_name (str): The name of the microphone node.
            conversation_node_name (str): The name of the conversation node.
        """
        super().__init__('speech_api_client')
        
        # Create service clients
        self.stt_client = self.create_client(SpeechToText, f"/{microphone_node_name}/speech_to_text")
        self.record_client = self.create_client(RecordAudio, f"/{microphone_node_name}/record_audio")
        self.transcription_mode_client = self.create_client(SetTranscriptionMode, f"/{microphone_node_name}/set_transcription_mode")
        self.llm_settings_client = self.create_client(SetLLMSettings, f"/{conversation_node_name}/set_llm_settings")
        self.llm_response_client = self.create_client(LLMResponse, f"/{conversation_node_name}/llm_response")
        self.llm_clear_history_client = self.create_client(Trigger, f"/{conversation_node_name}/clear_llm_history")
        self.say_client = self.create_client(Say, "/naoqi_speech_node/say")

        # Wait for services to be available
        self.wait_for_services()
        
        self.get_logger().info("Speech API initialized.")

    def wait_for_services(self):
        """Waits for all services to become available."""
        self.get_logger().info("Waiting for services...")
        
        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('speech_to_text service not available, waiting again...')
            
        while not self.record_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('record_audio service not available, waiting again...')
            
        while not self.transcription_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_transcription_mode service not available, waiting again...')
            
        while not self.llm_settings_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_llm_settings service not available, waiting again...')
            
        while not self.llm_response_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('llm_response service not available, waiting again...')
            
        while not self.llm_clear_history_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('clear_llm_history service not available, waiting again...')
            
        while not self.say_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('say service not available, waiting again...')
            
        self.get_logger().info("All services are available.")

    ##
    # Microphone Node Methods
    ##

    def listen(self, autocut=True, timeout=15.0) -> str | None:
        """
        Records audio until speech ends or a timeout is reached and returns the transcription.

        Args:
            autocut (bool): If True, automatically stops recording after speech.
            timeout (float): Maximum recording duration in seconds.

        Returns:
            The transcribed text as a string, or None if an error occurred.
        """
        self.get_logger().info(f"Calling speech_to_text service (autocut={autocut}, timeout={timeout})...")
        request = SpeechToText.Request()
        request.autocut = autocut
        request.timeout = int(timeout)
        
        future = self.stt_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Transcription received: '{response.transcription}'")
            return response.transcription
        else:
            self.get_logger().error("Failed to get response from speech_to_text service")
            return None

    def record_audio(self, file_name, duration=0.0) -> bool:
        """
        Records audio and saves it to a file on the server running the node.

        Args:
            file_name (str): The name of the file to save (without extension).
            duration (float): Duration to record. If 0, uses voice activity detection.

        Returns:
            True if recording was successful, False otherwise.
        """
        self.get_logger().info(f"Calling record_audio service (file='{file_name}', duration={duration})...")
        request = RecordAudio.Request()
        request.file_name = file_name
        request.duration = float(duration)
        
        future = self.record_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            return response.success
        else:
            self.get_logger().error("Failed to get response from record_audio service")
            return False

    def set_transcription_mode(self, enabled, language='en') -> tuple[bool, str] | None:
        """
        Enables or disables the continuous, real-time transcription mode.

        Args:
            enabled (bool): True to enable real-time transcription, False to disable.
            language (str): The language model to use (e.g., 'en', 'es').

        Returns:
            A tuple (success, message), or None on failure.
        """
        self.get_logger().info(f"Setting transcription mode to {enabled} for language '{language}'...")
        request = SetTranscriptionMode.Request()
        request.state = enabled
        request.language = language
        
        future = self.transcription_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            return response.success, response.message
        else:
            self.get_logger().error("Failed to get response from set_transcription_mode service")
            return None

    ##
    # Conversation Node Methods
    ##

    def set_llm_settings(self, context, model_name="gpt-4o", model_provider="-azure", temperature=0.7, max_tokens=256) -> bool:
        """
        Configures the settings for the Large Language Model.

        Args:
            context (str): The system prompt or context for the conversation.
            model_name (str): The name of the LLM model.
            model_provider (str): The provider of the model (appended to name).
            temperature (float): The creativity/randomness of the model's output.
            max_tokens (int): The maximum length of the response.

        Returns:
            True if settings were updated successfully, False otherwise.
        """
        self.get_logger().info("Updating LLM settings...")
        request = SetLLMSettings.Request()
        request.model_name = model_name
        request.model_provider = model_provider
        request.temperature = float(temperature)
        request.max_tokens = int(max_tokens)
        request.context = context
        
        future = self.llm_settings_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"LLM settings update status: {response.message}")
            return response.success
        else:
            self.get_logger().error("Failed to get response from set_llm_settings service")
            return False

    def ask(self, prompt) -> str:
        """
        Sends a prompt to the LLM and gets a response.

        Args:
            prompt (str): The question or statement to send to the LLM.

        Returns:
            The LLM's answer, or an error message if it failed.
        """
        self.get_logger().info(f"Sending prompt to LLM: '{prompt}'")
        request = LLMResponse.Request()
        request.prompt = prompt
        
        future = self.llm_response_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"LLM response received: '{response.answer}'")
            return response.answer
        else:
            self.get_logger().error("Failed to get response from llm_response service")
            return "Error: Failed to get a response from the LLM service."

    def clear_conversation_history(self) -> bool:
        """
        Clears the conversation history in the LLM.

        Returns:
            True if the history was cleared successfully, False otherwise.
        """
        self.get_logger().info("Clearing LLM conversation history...")
        request = Trigger.Request()
        
        future = self.llm_clear_history_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Clear history status: {response.message}")
            return response.success
        else:
            self.get_logger().error("Failed to get response from clear_llm_history service")
            return False

    def say(self, text_say: str, language_say="English", animated_say=False, asynchronous_say=True) -> bool:
        """
        Makes the robot say a text.

        Returns:
            True if the robot said the text correctly, False otherwise.
        """
        self.get_logger().info("Robot is going to start speaking")
        request = Say.Request()
        request.text = text_say
        request.language = language_say
        request.animated = animated_say
        request.asynchronous = asynchronous_say
        
        future = self.say_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Speaking status: {response.message}")
            return response.success
        else:
            self.get_logger().error("Failed to get response from say service")
            return False