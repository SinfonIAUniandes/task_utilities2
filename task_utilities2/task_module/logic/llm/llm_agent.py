# filepath: /home/cuevas/github_cosas/task_utilities2/task_utilities2/task_module/logic/llm/llm_agent.py
from typing import Optional, Dict, Any, List, Callable
import os
from datetime import datetime

# Importar clases de lógica interna
from .llm_settings import LLMSettings
from .llm_memory import LLMMemory
from dotenv import load_dotenv

# Cargar variables de entorno desde un archivo .env si existe
load_dotenv()

# Importar adaptadores de LangChain para agentes
try:
    from langchain_community.chat_models import AzureChatOpenAI, ChatOpenAI
    from langchain_ollama import ChatOllama
    from langchain.tools import Tool
    from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, BaseMessage
    from langgraph.prebuilt import create_react_agent
except ImportError:
    print("Advertencia: Faltan dependencias de LangChain para agentes. Instálalas con 'pip install langchain langchain-openai langchain-community langchain-ollama langgraph python-dotenv'")
    AzureChatOpenAI, ChatOpenAI, ChatOllama = None, None, None
    Tool = None
    AIMessage, HumanMessage, SystemMessage, BaseMessage = None, None, None, None
    create_react_agent = None

# Usar el mismo registro de modelos que el LLMHandler
MODEL_REGISTRY: Dict[str, Dict[str, Any]] = {
    "gpt-4o-azure": {
        "provider": "azure",
        "deployment_name": "GPT-4o",
        "AZURE_OPENAI_API_KEY": os.getenv("AZURE_OPENAI_API_KEY"),
        "AZURE_OPENAI_ENDPOINT": os.getenv("AZURE_OPENAI_ENDPOINT"),
        "api_version": os.getenv("AZURE_OPENAI_API_VERSION"),
    },
    "gpt-4.1-azure": {
        "provider": "azure",
        "deployment_name": "gpt-4.1",
        "AZURE_OPENAI_API_KEY": os.getenv("AZURE_OPENAI_API_KEY"),
        "AZURE_OPENAI_ENDPOINT": os.getenv("AZURE_OPENAI_ENDPOINT"),
        "api_version": os.getenv("AZURE_OPENAI_API_VERSION"),
    },
    "llama3.1-local": {
        "provider": "ollama",
        "model": "llama3.1",
        "base_url": os.getenv("OLLAMA_BASE_URL", "http://localhost:11434"),
    },
    "gpt-4o-mini": {
        "provider": "openai",
        "model": "gpt-4o-mini",
        # La API key es leída automáticamente por ChatOpenAI desde la variable
        # de entorno OPENAI_API_KEY.
    }
}

# Herramientas predeterminadas que pueden ser utilizadas por el agente
def get_current_time(query: str = "") -> str:
    """Devuelve la hora actual en formato HH:MM:SS."""
    return datetime.now().strftime("%H:%M:%S")

def get_current_date(query: str = "") -> str:
    """Devuelve la fecha actual en formato DD/MM/YYYY."""
    return datetime.now().strftime("%d/%m/%Y")

def get_current_datetime(query: str = "") -> str:
    """Devuelve la fecha y hora actual en formato completo."""
    return datetime.now().strftime("%d/%m/%Y %H:%M:%S")

# Registro de herramientas predeterminadas
DEFAULT_TOOLS: List[Tool] = [
    Tool(
        name="get_current_time",
        func=get_current_time,
        description="Útil cuando necesitas saber la hora actual. No requiere parámetros de entrada.",
    ),
    Tool(
        name="get_current_date", 
        func=get_current_date,
        description="Útil cuando necesitas saber la fecha actual. No requiere parámetros de entrada.",
    ),
    Tool(
        name="get_current_datetime",
        func=get_current_datetime,
        description="Útil cuando necesitas saber la fecha y hora actuales. No requiere parámetros de entrada.",
    )
]

class LLMAgent:
    """
    Clase orquestadora para interactuar con un agente ReAct LLM.
    Gestiona la configuración, la memoria, las herramientas y la creación de agentes LLM.
    """
    def __init__(self, initial_settings: Optional[Dict[str, Any]] = None, tools: Optional[List[Tool]] = None):
        if not all([AzureChatOpenAI, ChatOpenAI, ChatOllama, create_react_agent]):
            raise ImportError("Las dependencias de LangChain para agentes no están instaladas. Ejecuta: pip install langchain langchain-openai langchain-community langchain-ollama langgraph python-dotenv")

        self.settings = LLMSettings()
        if initial_settings:
            # Establece un modelo por defecto si no se proporciona uno
            if "model_name" not in initial_settings:
                initial_settings["model_name"] = next(iter(MODEL_REGISTRY))
            self.settings.update_from_dict(initial_settings)

        self.memory = LLMMemory(self.settings.context)
        self.llm_client = None
        self.agent = None
        
        # Configurar herramientas: usar las proporcionadas o las predeterminadas
        self.tools = tools if tools is not None else DEFAULT_TOOLS.copy()
        
        self._recreate_agent()

    def _recreate_client(self):
        """
        Crea (o recrea) la instancia del cliente LLM basada en la configuración actual.
        """
        model_name = self.settings.model_name
        if model_name not in MODEL_REGISTRY:
            print(f"Error: Modelo '{model_name}' no encontrado en MODEL_REGISTRY.")
            self.llm_client = None
            return

        config = MODEL_REGISTRY[model_name]
        provider = config.get("provider")

        llm_kwargs = {
            "temperature": self.settings.temperature,
            "max_tokens": self.settings.max_tokens,
        }

        print(f"Recreando cliente LLM para el agente con modelo '{model_name}' y proveedor '{provider}'...")

        try:
            if provider == "azure":
                self.llm_client = AzureChatOpenAI(
                    azure_deployment=config["deployment_name"],
                    openai_api_version=config["api_version"],
                    api_key=config["AZURE_OPENAI_API_KEY"],
                    azure_endpoint=config["AZURE_OPENAI_ENDPOINT"],
                    **llm_kwargs
                )
            elif provider == "ollama":
                # Ollama usa 'num_predict' en lugar de 'max_tokens'
                ollama_kwargs = llm_kwargs.copy()
                if "max_tokens" in ollama_kwargs:
                    ollama_kwargs["num_predict"] = ollama_kwargs.pop("max_tokens")
                self.llm_client = ChatOllama(
                    model=config["model"],
                    base_url=config["base_url"],
                    **ollama_kwargs
                )
            elif provider == "openai":
                self.llm_client = ChatOpenAI(
                    model=config["model"],
                    **llm_kwargs
                )
            else:
                raise ValueError(f"Proveedor '{provider}' no soportado.")
            print("Cliente LLM para agente creado exitosamente.")
        except Exception as e:
            print(f"Error al crear el cliente LLM para agente '{model_name}': {e}")
            self.llm_client = None

    def _recreate_agent(self):
        """
        Crea (o recrea) el agente ReAct basado en la configuración actual.
        """
        self._recreate_client()
        
        if not self.llm_client:
            print("Error: No se puede crear el agente sin un cliente LLM válido.")
            self.agent = None
            return

        try:
            # Crear el agente ReAct usando LangGraph
            self.agent = create_react_agent(
                self.llm_client,
                tools=self.tools,
                prompt=self._create_prompt
            )
            print("Agente ReAct creado exitosamente.")
        except Exception as e:
            print(f"Error al crear el agente ReAct: {e}")
            self.agent = None

    def _create_prompt(self, state):
        """Create the prompt for the React agent."""
        messages = state.get("messages", [])
        
        # Add system message if context is provided
        if self.settings.context:
            system_prompt = [{
                "role": "system", 
                "content": self.settings.context
            }]
            return system_prompt + messages
        
        return messages

    def add_tool(self, tool: Tool):
        """
        Añade una herramienta al agente.
        """
        self.tools.append(tool)
        self._recreate_agent()  # Recrear el agente con las nuevas herramientas
        print(f"Herramienta '{tool.name}' añadida al agente.")

    def add_custom_tool(self, name: str, func: Callable, description: str):
        """
        Añade una herramienta personalizada al agente.
        """
        tool = Tool(name=name, func=func, description=description)
        self.add_tool(tool)

    def remove_tool(self, tool_name: str):
        """
        Elimina una herramienta del agente por su nombre.
        """
        self.tools = [tool for tool in self.tools if tool.name != tool_name]
        self._recreate_agent()  # Recrear el agente sin la herramienta eliminada
        print(f"Herramienta '{tool_name}' eliminada del agente.")

    def list_tools(self) -> List[str]:
        """
        Devuelve una lista con los nombres de todas las herramientas disponibles.
        """
        return [tool.name for tool in self.tools]

    def update_settings(self, new_settings: Dict[str, Any]):
        """
        Actualiza la configuración del agente y lo recrea si es necesario.
        """
        print(f"Actualizando configuración del agente con: {new_settings}")
        self.settings.update_from_dict(new_settings)
        self.memory.set_context(self.settings.context)
        self._recreate_agent()
        print("Configuración del agente actualizada y agente recreado.")

    def invoke_agent(self, input_text: str) -> Dict[str, Any]:
        """
        Invoca al agente ReAct con un input y devuelve la respuesta completa.
        """
        if not self.agent:
            error_msg = "Error: El agente no está inicializado. Verifica la configuración y las credenciales."
            print(error_msg)
            return {"error": error_msg}

        # Agregar el mensaje del usuario a la memoria
        self.memory.add_message("user", input_text)
        
        try:
            print(f"Invocando agente con: {input_text}")
            
            # Create state with message history
            state = {
                "messages": [
                    {"role": "user", "content": input_text}
                ]
            }
            
            # Invoke the agent with the state
            result = self.agent.invoke(state)
            
            # Extract the final message from the agent
            if "messages" in result and result["messages"]:
                final_message = result["messages"][-1]
                if hasattr(final_message, 'content'):
                    output = final_message.content
                else:
                    output = str(final_message)
            else:
                output = "No se pudo obtener respuesta del agente."
            
            # Agregar la respuesta del agente a la memoria
            self.memory.add_message("assistant", output)
            
            # Return formatted response
            return {
                "output": output,
                "messages": result.get("messages", []),
                "state": result
            }
            
        except Exception as e:
            error_msg = f"Error durante la invocación del agente: {e}"
            print(error_msg)
            # Eliminar el último mensaje de usuario del historial si la llamada falla
            history = self.memory.get_history()
            if history and history[-1]["role"] == "user":
                history.pop()
            return {"error": error_msg}

    def invoke_with_structured_response(self, input_text: str, expected_format: str = "json") -> Dict[str, Any]:
        """
        Invoca al agente y intenta parsear una respuesta estructurada.
        Similar al patrón usado en evaluation-agent.py
        """
        result = self.invoke_agent(input_text)
        
        if "error" in result:
            return result
            
        output = result.get("output", "")
        
        if expected_format == "json":
            try:
                # Try to extract JSON from the response
                if "```json" in output:
                    json_start = output.find("```json") + 7
                    json_end = output.find("```", json_start)
                    json_str = output[json_start:json_end].strip()
                else:
                    # Look for JSON-like structure
                    json_start = output.find("{")
                    json_end = output.rfind("}") + 1
                    if json_start >= 0 and json_end > json_start:
                        json_str = output[json_start:json_end]
                    else:
                        json_str = output
                
                import json
                parsed_result = json.loads(json_str)
                
                return {
                    "success": True,
                    "parsed_response": parsed_result,
                    "raw_output": output,
                    "state": result.get("state", {})
                }
                
            except (json.JSONDecodeError, ValueError) as e:
                return {
                    "success": False,
                    "error": f"Failed to parse JSON response: {e}",
                    "raw_output": output,
                    "state": result.get("state", {})
                }
        
        # For non-JSON formats, return raw output
        return {
            "success": True,
            "raw_output": output,
            "state": result.get("state", {})
        }

    def get_response(self, prompt: str) -> str:
        """
        Método de conveniencia que devuelve solo la respuesta de texto del agente.
        """
        result = self.invoke_agent(prompt)
        if "error" in result:
            return result["error"]
        
        output = result.get("output", "")
        
        # Try to extract clean text response if it's wrapped in JSON or other formats
        try:
            if output.strip().startswith('```') and output.strip().endswith('```'):
                # Remove code block markers
                lines = output.strip().split('\n')
                if len(lines) > 2:
                    output = '\n'.join(lines[1:-1])
            elif output.strip().startswith('{') and output.strip().endswith('}'):
                # Try to parse JSON and extract meaningful content
                import json
                json_data = json.loads(output)
                if isinstance(json_data, dict):
                    # Look for common response fields
                    for field in ['response', 'message', 'answer', 'result', 'output']:
                        if field in json_data:
                            output = str(json_data[field])
                            break
        except (json.JSONDecodeError, ValueError):
            # If parsing fails, use original output
            pass
            
        return output or "No se pudo obtener respuesta del agente."

    def clear_history(self):
        """
        Limpia el historial de la conversación.
        """
        print("Limpiando historial de conversación del agente.")
        self.memory.clear()

    def get_history(self) -> List[Dict[str, str]]:
        """
        Devuelve el historial completo de la conversación.
        """
        return self.memory.get_history()

class ModernLLMAgentWrapper:
    """
    Wrapper class for the modernized LLM agent, following the pattern from evaluation-agent.py.
    Provides a cleaner interface and better error handling.
    """
    
    def __init__(self, initial_settings: Optional[Dict[str, Any]] = None, tools: Optional[List[Tool]] = None):
        """Initialize the modern LLM agent wrapper."""
        self.agent = LLMAgent(initial_settings, tools)
    
    def chat(self, message: str) -> Dict[str, Any]:
        """
        Simple chat interface that returns a structured response.
        
        Args:
            message: The user message/prompt
            
        Returns:
            Dictionary with response information
        """
        try:
            result = self.agent.invoke_agent(message)
            
            if "error" in result:
                return {
                    "success": False,
                    "error": result["error"],
                    "response": "Sorry, I encountered an error processing your request."
                }
            
            return {
                "success": True,
                "response": result.get("output", "No response generated"),
                "raw_result": result
            }
            
        except Exception as e:
            return {
                "success": False,
                "error": f"Unexpected error: {str(e)}",
                "response": "Sorry, I encountered an unexpected error."
            }
    
    def process_with_tools(self, message: str, expected_format: str = "text") -> Dict[str, Any]:
        """
        Process a message using the agent's tools with structured output.
        
        Args:
            message: The user message/prompt
            expected_format: Expected format of the response ("text", "json")
            
        Returns:
            Dictionary with structured response
        """
        try:
            if expected_format == "json":
                result = self.agent.invoke_with_structured_response(message, "json")
            else:
                result = self.agent.invoke_agent(message)
                if "error" not in result:
                    result = {
                        "success": True,
                        "raw_output": result.get("output", ""),
                        "state": result
                    }
                else:
                    result = {
                        "success": False,
                        "error": result["error"]
                    }
            
            return result
            
        except Exception as e:
            return {
                "success": False,
                "error": f"Processing error: {str(e)}"
            }
    
    def add_tool(self, tool: Tool):
        """Add a tool to the agent."""
        self.agent.add_tool(tool)
    
    def add_custom_tool(self, name: str, func: Callable, description: str):
        """Add a custom tool to the agent."""
        self.agent.add_custom_tool(name, func, description)
    
    def list_tools(self) -> List[str]:
        """List available tools."""
        return self.agent.list_tools()
    
    def clear_history(self):
        """Clear conversation history."""
        self.agent.clear_history()
    
    def get_history(self) -> List[Dict[str, str]]:
        """Get conversation history."""
        return self.agent.get_history()
    
    def update_settings(self, new_settings: Dict[str, Any]):
        """Update agent settings."""
        self.agent.update_settings(new_settings)