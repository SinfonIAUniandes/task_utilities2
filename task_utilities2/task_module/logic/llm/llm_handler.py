from typing import Optional, Dict, Any, List
import os

# Importar clases de lógica interna
from .llm_settings import LLMSettings
from .llm_memory import LLMMemory
from dotenv import load_dotenv

# Cargar variables de entorno desde un archivo .env si existe
load_dotenv()

# Importar adaptadores de LangChain
try:
    from langchain_community.chat_models import AzureChatOpenAI, ChatOpenAI
    from langchain_ollama import ChatOllama
    from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, BaseMessage
except ImportError:
    print("Advertencia: Faltan dependencias de LangChain. Instálalas con 'pip install langchain langchain-openai langchain-community langchain-ollama python-dotenv'")
    AzureChatOpenAI, ChatOpenAI, ChatOllama = None, None, None
    AIMessage, HumanMessage, SystemMessage, BaseMessage = None, None, None, None


# Registro de modelos que busca la configuración en variables de entorno.
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

class LLMHandler:
    """
    Clase orquestadora para interactuar con un LLM.
    Gestiona la configuración, la memoria y la creación de clientes LLM.
    """
    def __init__(self, initial_settings: Optional[Dict[str, Any]] = None):
        if not all([AzureChatOpenAI, ChatOpenAI, ChatOllama]):
             raise ImportError("Las dependencias de LangChain no están instaladas. Ejecuta: pip install langchain langchain-openai langchain-community langchain-ollama python-dotenv")

        self.settings = LLMSettings()
        if initial_settings:
            # Establece un modelo por defecto si no se proporciona uno
            if "model_name" not in initial_settings:
                initial_settings["model_name"] = next(iter(MODEL_REGISTRY))
            self.settings.update_from_dict(initial_settings)

        self.memory = LLMMemory(self.settings.context)
        self.llm_client = None
        self._recreate_client()

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

        print(f"Recreando cliente para el modelo '{model_name}' con el proveedor '{provider}'...")

        try:
            if provider == "azure":
                self.llm_client = AzureChatOpenAI(
                    azure_deployment=config["deployment_name"], openai_api_version=config["api_version"], api_key=config["AZURE_OPENAI_API_KEY"], azure_endpoint=config["AZURE_OPENAI_ENDPOINT"]
                )
            elif provider == "ollama":
                # Ollama usa 'num_predict' en lugar de 'max_tokens'
                ollama_kwargs = llm_kwargs.copy()
                if "max_tokens" in ollama_kwargs:
                    ollama_kwargs["num_predict"] = ollama_kwargs.pop("max_tokens")
                self.llm_client = ChatOllama(
                    model=config["model"], base_url=config["base_url"]
                )
            elif provider == "openai":
                self.llm_client = ChatOpenAI(model=config["model"])
            else:
                raise ValueError(f"Proveedor '{provider}' no soportado.")
            print("Cliente LLM creado exitosamente.")
        except Exception as e:
            print(f"Error al crear el cliente LLM para '{model_name}': {e}")
            self.llm_client = None


    def update_settings(self, new_settings: Dict[str, Any]):
        """
        Actualiza la configuración del LLM y recrea el cliente si es necesario.
        """
        print(f"Actualizando configuración con: {new_settings}")
        self.settings.update_from_dict(new_settings)
        self.memory.set_context(self.settings.context)
        self._recreate_client()
        print("Configuración actualizada y cliente recreado.")

    def get_response(self, prompt: str) -> str:
        """
        Envía un prompt al LLM, gestiona el historial y devuelve la respuesta.
        """
        if not self.llm_client:
            error_msg = "Error: El cliente LLM no está inicializado. Verifica la configuración y las credenciales."
            print(error_msg)
            return error_msg

        self.memory.add_message("user", prompt)
        
        # Convertir el historial a formato LangChain
        history_langchain: List[BaseMessage] = []
        for msg in self.memory.get_history():
            if msg["role"] == "system":
                history_langchain.append(SystemMessage(content=msg["content"]))
            elif msg["role"] == "user":
                history_langchain.append(HumanMessage(content=msg["content"]))
            elif msg["role"] == "assistant":
                history_langchain.append(AIMessage(content=msg["content"]))

        try:
            print(f"Enviando al LLM: {history_langchain}")
            response_message = self.llm_client.invoke(history_langchain)
            answer = response_message.content
        except Exception as e:
            error_msg = f"Error durante la invocación del LLM: {e}"
            print(error_msg)
            # Eliminar el último mensaje de usuario del historial si la llamada falla
            self.memory.get_history().pop()
            return error_msg
        
        self.memory.add_message("assistant", answer)
        return answer

    def clear_history(self):
        """
        Limpia el historial de la conversación.
        """
        print("Limpiando historial de conversación.")
        self.memory.clear()
        
# Ejemplo de cómo se podría usar
if __name__ == '__main__':
    # Asegúrate de tener un archivo .env con las credenciales o tenerlas
    # exportadas en tu terminal. Por ejemplo, para Azure:
    # AZURE_OPENAI_CHAT_DEPLOYMENT="gpt-4"
    # AZURE_OPENAI_ENDPOINT="https://tu-endpoint.openai.azure.com/"
    # AZURE_OPENAI_API_KEY="tu-api-key"
    # AZURE_OPENAI_API_VERSION="2024-02-01"

    print("--- Probando LLMHandler con Azure ---")
    try:
        handler = LLMHandler({"model_name": "gpt-4-azure"})
        
        respuesta1 = handler.get_response("Hola, ¿cómo estás?")
        print(f"Respuesta del LLM: {respuesta1}\n")

        respuesta2 = handler.get_response("¿Cuál fue mi primera pregunta?")
        print(f"Respuesta del LLM: {respuesta2}\n")

        handler.update_settings({
            "temperature": 0.9,
            "context": "Eres un pirata malhumorado que solo habla con jerga pirata."
        })

        respuesta3 = handler.get_response("Y ahora, ¿quién eres?")
        print(f"Respuesta del LLM: {respuesta3}\n")
    except Exception as e:
        print(f"No se pudo ejecutar el ejemplo de Azure: {e}")