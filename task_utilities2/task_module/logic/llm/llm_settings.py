from dataclasses import dataclass, field
from typing import Optional, Dict, Any

@dataclass
class LLMSettings:
    """
    Almacena la configuración para un cliente LLM.
    """
    model_name: str = "default_model"
    temperature: float = 0.7
    max_tokens: int = 500
    context: Optional[str] = None

    def update_from_dict(self, settings_dict: Dict[str, Any]):
        """
        Actualiza los atributos de la clase desde un diccionario.
        """
        for key, value in settings_dict.items():
            if hasattr(self, key):
                # Realiza conversiones de tipo si es necesario
                try:
                    if key == "temperature":
                        value = float(value)
                    elif key == "max_tokens":
                        value = int(value)
                    setattr(self, key, value)
                except (ValueError, TypeError) as e:
                    print(f"Error al convertir el ajuste '{key}' con valor '{value}': {e}")

# Ejemplo de cómo se podría usar
if __name__ == '__main__':
    settings = LLMSettings()
    print(f"Ajustes iniciales: {settings}")

    new_settings = {
        "model_name": "gpt-4o-azure",
        "temperature": "0.1",
        "max_tokens": "1024",
        "context": "Eres un asistente robótico servicial."
    }
    settings.update_from_dict(new_settings)
    print(f"Ajustes actualizados: {settings}")