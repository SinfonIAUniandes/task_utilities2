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
    from langchain.agents import AgentExecutor, create_react_agent
    from langchain import hub
    from langchain.tools import Tool
    from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, BaseMessage
    from langchain.prompts import PromptTemplate
except ImportError:
    print("Advertencia: Faltan dependencias de LangChain para agentes. Instálalas con 'pip install langchain langchain-openai langchain-community langchain-ollama langchain-hub python-dotenv'")
    AzureChatOpenAI, ChatOpenAI, ChatOllama = None, None, None
    AgentExecutor, create_react_agent, hub, Tool = None, None, None, None
    AIMessage, HumanMessage, SystemMessage, BaseMessage = None, None, None, None
    PromptTemplate = None

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
        if not all([AzureChatOpenAI, ChatOpenAI, ChatOllama, AgentExecutor, create_react_agent]):
            raise ImportError("Las dependencias de LangChain para agentes no están instaladas. Ejecuta: pip install langchain langchain-openai langchain-community langchain-ollama langchain-hub python-dotenv")

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
        
        # Configuración para el prompt del agente ReAct
        self.verbose = False
        
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
            # Crear el agente ReAct
            self.agent = create_react_agent(self.llm_client, self.tools, self.settings.context, verbose=self.verbose)

            print("Agente ReAct creado exitosamente.")
        except Exception as e:
            print(f"Error al crear el agente ReAct: {e}")
            self.agent = None

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
            response = self.agent.invoke({"input": input_text})
            
            # Extraer la respuesta del agente
            output = response.get("output", "No se pudo obtener respuesta del agente.")
            
            # Agregar la respuesta del agente a la memoria
            self.memory.add_message("assistant", output)
            
            return response
        except Exception as e:
            error_msg = f"Error durante la invocación del agente: {e}"
            print(error_msg)
            # Eliminar el último mensaje de usuario del historial si la llamada falla
            history = self.memory.get_history()
            if history and history[-1]["role"] == "user":
                history.pop()
            return {"error": error_msg}

    def get_response(self, prompt: str) -> str:
        """
        Método de conveniencia que devuelve solo la respuesta de texto del agente.
        """
        result = self.invoke_agent(prompt)
        if "error" in result:
            return result["error"]
        return result.get("output", "No se pudo obtener respuesta del agente.")

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