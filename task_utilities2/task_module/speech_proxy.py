import sys
from threading import Thread

from speech_msgs2.srv import SpeechToText, RecordAudio, SetTranscriptionMode
from speech_msgs2.srv import SetLLMSettings, LLMResponse
from naoqi_utilities_msgs.srv import Say
from std_srvs.srv import Trigger


class SpeechProxy:
    """
    Modified Speech proxy that uses a parent node instead of creating its own.
    This allows TaskModule to be the single ROS2 node managing all services.
    """
    
    def __init__(self, parent_node, microphone_node_name="microphone_node", conversation_node_name="conversation_node"):
        """
        Initialize Speech proxy with a parent node.
        
        Args:
            parent_node (Node): The parent ROS2 node to use for services
            microphone_node_name (str): Name of the microphone node
            conversation_node_name (str): Name of the conversation node
        """
        self.node = parent_node
        self.microphone_node_name = microphone_node_name
        self.conversation_node_name = conversation_node_name
        
        # Create service clients using the parent node
        self.stt_client = self.node.create_client(SpeechToText, f"/{microphone_node_name}/speech_to_text")
        self.record_client = self.node.create_client(RecordAudio, f"/{microphone_node_name}/record_audio")
        self.transcription_mode_client = self.node.create_client(SetTranscriptionMode, f"/{microphone_node_name}/set_transcription_mode")
        self.llm_settings_client = self.node.create_client(SetLLMSettings, f"/{conversation_node_name}/set_llm_settings")
        self.llm_response_client = self.node.create_client(LLMResponse, f"/{conversation_node_name}/llm_response")
        self.llm_clear_history_client = self.node.create_client(Trigger, f"/{conversation_node_name}/clear_llm_history")
        self.say_client = self.node.create_client(Say, "/naoqi_speech_node/say")
        
        # Wait for services to be available
        self.wait_for_services()
        
        self.node.get_logger().info("SpeechProxy initialized")
    
    def wait_for_services(self):
        """Wait for all services to become available."""
        self.node.get_logger().info("Waiting for speech services...")
        
        services = [
            (self.stt_client, 'speech_to_text'),
            (self.record_client, 'record_audio'),
            (self.transcription_mode_client, 'set_transcription_mode'),
            (self.llm_settings_client, 'set_llm_settings'),
            (self.llm_response_client, 'llm_response'),
            (self.llm_clear_history_client, 'clear_llm_history'),
            (self.say_client, 'say')
        ]
        
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(f'{service_name} service not available, waiting again...')
        
        self.node.get_logger().info("All speech services are available")
    
    # Microphone Node Methods
    def listen(self, autocut=True, timeout=15.0) -> str | None:
        """
        Records audio until speech ends or a timeout is reached and returns the transcription.
        
        Args:
            autocut (bool): If True, automatically stops recording after speech.
            timeout (float): Maximum recording duration in seconds.
            
        Returns:
            The transcribed text as a string, or None if an error occurred.
        """
        from speech_msgs2.srv import SpeechToText
        
        self.node.get_logger().info(f"Calling speech_to_text service (autocut={autocut}, timeout={timeout})...")
        request = SpeechToText.Request()
        request.autocut = autocut
        request.timeout = int(timeout)
        
        response = self.stt_client.call(request)
        return response.transcription if response else None
    
    def record_audio(self, file_name, duration=0.0) -> bool:
        """
        Records audio and saves it to a file on the server running the node.
        
        Args:
            file_name (str): The name of the file to save (without extension).
            duration (float): Duration to record. If 0, uses voice activity detection.
            
        Returns:
            True if recording was successful, False otherwise.
        """
        from speech_msgs2.srv import RecordAudio
        
        self.node.get_logger().info(f"Calling record_audio service (file='{file_name}', duration={duration})...")
        request = RecordAudio.Request()
        request.file_name = file_name
        request.duration = float(duration)
        
        response = self.record_client.call(request)
        return response.success if response else False
    
    def set_transcription_mode(self, enabled, language='en') -> bool:
        """
        Enables or disables the continuous, real-time transcription mode.
        
        Args:
            enabled (bool): True to enable real-time transcription, False to disable.
            language (str): The language model to use (e.g., 'en', 'es').
            
        Returns:
            True if successful, False otherwise.
        """
        from speech_msgs2.srv import SetTranscriptionMode
        
        self.node.get_logger().info(f"Setting transcription mode to {enabled} for language '{language}'...")
        request = SetTranscriptionMode.Request()
        request.state = enabled
        request.language = language
        
        response = self.transcription_mode_client.call(request)
        return response.success if response else False
    
    # Conversation Node Methods
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
        from speech_msgs2.srv import SetLLMSettings
        
        self.node.get_logger().info("Updating LLM settings...")
        request = SetLLMSettings.Request()
        request.model_name = model_name
        request.model_provider = model_provider
        request.temperature = float(temperature)
        request.max_tokens = int(max_tokens)
        request.context = context
        
        response = self.llm_settings_client.call(request)
        return response.success if response else False
    
    def ask(self, prompt) -> str:
        """
        Sends a prompt to the LLM and gets a response.
        
        Args:
            prompt (str): The question or statement to send to the LLM.
            
        Returns:
            The LLM's answer, or an error message if it failed.
        """
        from speech_msgs2.srv import LLMResponse
        
        self.node.get_logger().info(f"Sending prompt to LLM: '{prompt}'")
        request = LLMResponse.Request()
        request.prompt = prompt
        
        response = self.llm_response_client.call(request)
        return response.answer if response else "Error: No response from LLM"
    
    def clear_conversation_history(self) -> bool:
        """
        Clears the conversation history in the LLM.
        
        Returns:
            True if the history was cleared successfully, False otherwise.
        """
        from std_srvs.srv import Trigger
        
        self.node.get_logger().info("Clearing LLM conversation history...")
        request = Trigger.Request()
        
        response = self.llm_clear_history_client.call(request)
        return response.success if response else False
    
    def say(self, text_say: str, language_say="English", animated_say=False, asynchronous_say=False) -> bool:
        """
        Makes the robot say a text.
        
        Args:
            text_say (str): Text to be spoken
            language_say (str): Language for speech
            animated_say (bool): Whether to use animated speech
            asynchronous_say (bool): Whether to speak asynchronously
        
        Returns:
            True if the robot said the text correctly, False otherwise.
        """
        from naoqi_utilities_msgs.srv import Say
        
        request = Say.Request()
        request.text = text_say
        request.language = language_say
        request.animated = animated_say
        request.asynchronous = asynchronous_say
        
        response = self.say_client.call(request)
        return response.success if response else False
    
    def create_subscription(self, msg_type, topic, callback, qos_profile):
        """
        Create a subscription using the parent node.
        
        This allows the speech proxy to create subscriptions for real-time features
        while using the main TaskModule node.
        """
        return self.node.create_subscription(msg_type, topic, callback, qos_profile)
