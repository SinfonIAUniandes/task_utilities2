#!/usr/bin/env python3
import rclpy
import threading
import time
from rclpy.node import Node

# Note: This script assumes that the 'speech_msgs2' and 'std_srvs' packages,
# which define the service types, are available in your ROS2 workspace.
from speech_msgs2.srv import SpeechToText, RecordAudio, SetTranscriptionMode
from speech_msgs2.srv import SetLLMSettings, LLMResponse
from naoqi_utilities_msgs.srv import Say
from std_srvs.srv import Trigger


class Speech:
    """
    A Python class to interact with speech and conversation ROS2 nodes.
    
    This class can either create its own ROS2 node for simple, standalone use
    or accept an existing node object to be part of a larger application.
    """

    def __init__(self,
                 node: Node | None = None,
                 microphone_node_name="microphone_node",
                 conversation_node_name="conversation_node",
                 wait_for_services=True,
                 timeout_sec=5.0):
        """
        Initializes the Speech API object.

        Args:
            node (rclpy.node.Node | None): An existing ROS2 node. If provided,
                the class will use it for all communications. If None, a new
                node will be created and managed internally.
            microphone_node_name (str): The name of the microphone node.
            conversation_node_name (str): The name of the conversation node.
            wait_for_services (bool): If True, blocks until all services are available.
            timeout_sec (float): Timeout in seconds for waiting for services.
        """
        self._external_node = node is not None
        self._executor = None
        
        if self._external_node:
            self._node = node
        else:
            # If no node is passed, create and manage one internally.
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node("speech_api_client")
            self._executor = rclpy.executors.SingleThreadedExecutor()
            self._executor.add_node(self._node)
            # Spin the internal executor in a thread to not block the main application
            threading.Thread(target=self._executor.spin, daemon=True).start()

        self._logger = self._node.get_logger()
        
        # --- Client definitions ---
        clients = {
            'stt': (SpeechToText, f"/{microphone_node_name}/speech_to_text"),
            'record': (RecordAudio, f"/{microphone_node_name}/record_audio"),
            'transcription_mode': (SetTranscriptionMode, f"/{microphone_node_name}/set_transcription_mode"),
            'llm_settings': (SetLLMSettings, f"/{conversation_node_name}/set_llm_settings"),
            'llm_response': (LLMResponse, f"/{conversation_node_name}/llm_response"),
            'llm_clear_history': (Trigger, f"/{conversation_node_name}/clear_llm_history"),
            'say': (Say, f"/naoqi_speech_node/say"),
        }

        self._clients = {}
        for name, (srv_type, srv_name) in clients.items():
            self._clients[name] = self._node.create_client(srv_type, srv_name)

        if wait_for_services:
            self.wait_for_all_services(timeout_sec)
        
        self._logger.info(f"Speech API initialized. Using {'external' if self._external_node else 'internal'} node.")

    def wait_for_all_services(self, timeout_sec=5.0):
        """Waits for all defined services to become available."""
        self._logger.info("Waiting for services...")
        for name, client in self._clients.items():
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self._logger.error(f"Service '{client.srv_name}' not available after {timeout_sec}s.")
        self._logger.info("All services are available.")

    def _call_service(self, client_name, request, timeout_sec=60.0):
        """Generic helper method to call a service and wait for the response."""
        client = self._clients.get(client_name)
        if not client or not client.service_is_ready():
            self._logger.error(f"Service '{client.srv_name if client else client_name}' is not available.")
            return None

        # The calling application is responsible for spinning the node so that
        # the future can complete.
        future = client.call_async(request)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout_sec:
                self._logger.error(f"Service call for '{client.srv_name}' timed out after {timeout_sec}s.")
                return None
            time.sleep(0.05)
        
        try:
            return future.result()
        except Exception as e:
            self._logger.error(f"Service call for '{client.srv_name}' failed: {e}")
            return None

    def shutdown(self):
        """
        Shuts down resources created by this instance.
        If an external node was provided, this method does nothing to the node.
        """
        if not self._external_node:
            self._logger.info("Shutting down internally managed Speech API client node.")
            if self._executor:
                self._executor.shutdown()
            if self._node:
                self._node.destroy_node()
        else:
            self._logger.info("Speech API instance detached. Node lifecycle is managed externally.")

    # ... (All other methods like listen, record_audio, ask, etc., remain exactly the same) ...

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
        self._logger.info(f"Calling speech_to_text service (autocut={autocut}, timeout={timeout})...")
        request = SpeechToText.Request(autocut=autocut, timeout=int(timeout))
        response = self._call_service('stt', request)
        if response:
            self._logger.info(f"Transcription received: '{response.transcription}'")
            return response.transcription
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
        self._logger.info(f"Calling record_audio service (file='{file_name}', duration={duration})...")
        request = RecordAudio.Request(file_name=file_name, duration=float(duration))
        response = self._call_service('record', request)
        return response.success if response else False

    def set_transcription_mode(self, enabled, language='en') -> tuple[bool, str] | None:
        """
        Enables or disables the continuous, real-time transcription mode.

        Args:
            enabled (bool): True to enable real-time transcription, False to disable.
            language (str): The language model to use (e.g., 'en', 'es').

        Returns:
            A tuple (success, message), or None on failure.
        """
        self._logger.info(f"Setting transcription mode to {enabled} for language '{language}'...")
        request = SetTranscriptionMode.Request(state=enabled, language=language)
        response = self._call_service('transcription_mode', request)
        if response:
            return response.success, response.message
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
        self._logger.info("Updating LLM settings...")
        request = SetLLMSettings.Request(
            model_name=model_name,
            model_provider=model_provider,
            temperature=float(temperature),
            max_tokens=int(max_tokens),
            context=context
        )
        response = self._call_service('llm_settings', request)
        if response:
            self._logger.info(f"LLM settings update status: {response.message}")
            return response.success
        return False

    def ask(self, prompt) -> str:
        """
        Sends a prompt to the LLM and gets a response.

        Args:
            prompt (str): The question or statement to send to the LLM.

        Returns:
            The LLM's answer, or an error message if it failed.
        """
        self._logger.info(f"Sending prompt to LLM: '{prompt}'")
        request = LLMResponse.Request(prompt=prompt)
        response = self._call_service('llm_response', request)
        if response:
            self._logger.info(f"LLM response received: '{response.answer}'")
            return response.answer
        return "Error: Failed to get a response from the LLM service."

    def clear_conversation_history(self) -> bool:
        """
        Clears the conversation history in the LLM.

        Returns:
            True if the history was cleared successfully, False otherwise.
        """
        self._logger.info("Clearing LLM conversation history...")
        request = Trigger.Request()
        response = self._call_service('llm_clear_history', request)
        if response:
            self._logger.info(f"Clear history status: {response.message}")
            return response.success
        return False

    ##
    # Conversation Node Methods
    ##

    def say(self, text_say: str, language_say="English", animated_say=False, asynchronous_say=True) -> bool:
        """
        Makes the robot say a text.

        Returns:
            True if the robot said the text correctly, False otherwise.
        """
        self._logger.info("Robot is going to start speaking")
        request = Say.Request(text=text_say, language=language_say, animated=animated_say, asynchronous=asynchronous_say)
        response = self._call_service('say', request,timeout_sec=10)
        
        if response:
            self._logger.info(f"Speaking status: {response.message}")
            return response.success
        return False