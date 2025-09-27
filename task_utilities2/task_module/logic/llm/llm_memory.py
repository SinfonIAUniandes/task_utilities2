from typing import List, Dict, Optional

class LLMMemory:
    """
    Gestiona el historial de la conversación para el LLM.
    """
    def __init__(self, system_context: Optional[str] = None):
        """
        Inicializa la memoria.
        :param system_context: Un mensaje de sistema inicial para guiar al LLM.
        """
        self._history: List[Dict[str, str]] = []
        self._system_context = system_context
        self.clear()

    def add_message(self, role: str, content: str):
        """
        Añade un mensaje al historial.
        :param role: El rol del autor del mensaje ('user', 'assistant').
        :param content: El contenido del mensaje.
        """
        if role not in ["user", "assistant"]:
            raise ValueError("El rol debe ser 'user' o 'assistant'.")
        self._history.append({"role": role, "content": content})

    def get_history(self) -> List[Dict[str, str]]:
        """
        Devuelve el historial completo de la conversación.
        """
        return self._history

    def clear(self):
        """
        Limpia el historial de la conversación, manteniendo el contexto del sistema si existe.
        """
        self._history.clear()
        if self._system_context:
            self._history.append({"role": "system", "content": self._system_context})

    def set_context(self, system_context: str):
        """
        Establece un nuevo contexto de sistema y limpia el historial.
        """
        self._system_context = system_context
        self.clear()

# Ejemplo de cómo se podría usar
if __name__ == '__main__':
    memory = LLMMemory(system_context="Eres un pirata.")
    print(f"Historial inicial: {memory.get_history()}")

    memory.add_message("user", "Hola, ¿quién eres?")
    memory.add_message("assistant", "¡Ahoy! Soy el Capitán Barbablanca.")
    print(f"Historial después de la conversación: {memory.get_history()}")

    memory.clear()
    print(f"Historial después de limpiar: {memory.get_history()}")

    memory.set_context("Ahora eres un chef francés.")
    print(f"Historial con nuevo contexto: {memory.get_history()}")