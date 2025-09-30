from langchain_openai import AzureChatOpenAI
from langgraph.prebuilt import create_react_agent
from langchain_core.tools import tool
from dotenv import load_dotenv
import os
import rclpy
from speech_msgs2.msg import Transcription
from threading import Thread
from task_utilities2.task_module.task_module import TaskModule
from langchain_core.messages import HumanMessage
import time
import random
from collections import deque

load_dotenv()

# Inicializar ROS2
rclpy.init()

# Crear instancia de TaskModule
task_module = TaskModule(
    node_name="realtime_robot_agent",
    robot_name="nova",
    enable_speech=True,
    enable_miscellaneous=True
)

@tool
def robot_speak(text: str) -> str:
    """Hace que el robot hable en voz alta."""
    try:
        task_module.speech.say(text, animated_say=True,language_say="Spanish")
        return f"Dicho con éxito: '{text}'"
    except Exception as e:
        return f"Error al hablar: {e}"

@tool
def change_eye_color(color_name: str) -> str:
    """Cambia el color de los ojos del robot. Puede usar nombres de colores o valores RGB."""
    try:
        color_presets = {
            "red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255),
            "yellow": (255, 255, 0), "purple": (128, 0, 128), "cyan": (0, 255, 255),
            "white": (255, 255, 255), "orange": (255, 165, 0), "pink": (255, 192, 203),
            "lime": (50, 205, 50)
        }
        if color_name.lower() in color_presets:
            r, g, b = color_presets[color_name.lower()]
        else:
            return f"Color desconocido '{color_name}'. Colores disponibles: {', '.join(color_presets.keys())}."
        
        r, g, b = max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b))
        task_module.miscellaneous.set_eye_color(red=r, green=g, blue=b, duration=0.0)
        return f"Color de ojos cambiado a {color_name} (RGB: {r}, {g}, {b})"
    except Exception as e:
        return f"Error al cambiar el color de los ojos: {e}"

@tool
def take_picture() -> str:
    """Hace que el robot tome una foto reproduciendo la animación de tomar foto."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/TakePicture_1")
        return "¡Foto tomada con éxito!"
    except Exception as e:
        return f"Error al tomar foto: {e}"

@tool
def get_current_time() -> str:
    """Obtiene la hora actual."""
    from datetime import datetime
    current_time = datetime.now().strftime("%H:%M:%S")
    return f"La hora actual es {current_time}"

@tool
def play_dance(dance_name: str) -> str:
    """Hace que el robot haga un baile pasado por parametro, las opciones son:
    arcadia
    asereje
    gangnamstyle
    macarena
    disco
    la_bamba
    """
    try:
        dance_name = dance_name.lower()
        if dance_name=="arcadia":
            chosen_dance = "Stand/arcadia/full_launcher"
        elif dance_name=="asereje":
            chosen_dance = "Stand/asereje/full_launcher"
        elif dance_name=="gangnamstyle":
            chosen_dance = "Stand/jgangnamstyle/full_launcher"
        elif dance_name=="macarena":
            chosen_dance = "Stand/Macarena/full_launcher"
        elif dance_name=="disco":
            chosen_dance = "Stand/disco/full_launcher"
        elif dance_name=="la_bamba":
            chosen_dance = "Stand/la_bamba/full_launcher"
        task_module.miscellaneous.play_animation(chosen_dance)
        return "¡Bailado con éxito!"
    except Exception as e:
        return f"Error al tomar foto: {e}"
    
@tool
def play_guitar() -> str:
    """Hace que el robot toque la guitarra"""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/AirGuitar_1")
        return "¡Guitarra tocada con éxito!"
    except Exception as e:
        return f"Error al tomar foto: {e}"
    
@tool
def throw_kisses() -> str:
    """Hace que el robot tire besos"""
    try:
        task_module.miscellaneous.play_animation("Stand/Gestures/Kisses_1")
        return "¡Besos lanzados con éxito!"
    except Exception as e:
        return f"Error al tomar foto: {e}"
    
@tool
def pose_for_picture() -> str:
    """Hace que el robot pose para una foto."""
    try:
        task_module.miscellaneous.play_animation("Stand/Gestures/ShowSky_8")
        return "¡Pose lograda con éxito!"
    except Exception as e:
        return f"Error al tomar foto: {e}"
    
@tool
def show_heart_gesture() -> str:
    """Hace que el robot muestre un gesto de 'Te Amo/Corazón'."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/LoveYou_1")
        return "¡Gesto de corazón mostrado con éxito!"
    except Exception as e:
        return f"Error al mostrar el gesto de corazón: {e}"

@tool
def make_a_call_gesture() -> str:
    """Hace que el robot reproduzca la animación de 'Llamar a alguien'."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/CallSomeone_1")
        return "¡Animación 'Llamar a alguien' reproducida con éxito!"
    except Exception as e:
        return f"Error al reproducir 'Llamar a alguien': {e}"

@tool
def act_like_a_zombie() -> str:
    """Hace que el robot reproduzca la animación de 'Zombi'."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/Zombie_1")
        return "¡Animación 'Zombi' reproducida con éxito!"
    except Exception as e:
        return f"Error al reproducir 'Zombi': {e}"

@tool
def drive_a_car_gesture() -> str:
    """Hace que el robot reproduzca la animación de 'Conducir Coche'."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/DriveCar_1")
        return "¡Animación 'Conducir Coche' reproducida con éxito!"
    except Exception as e:
        return f"Error al reproducir 'Conducir Coche': {e}"

@tool
def show_muscles() -> str:
    """Hace que el robot reproduzca la animación de 'Mostrar Músculos'."""
    try:
        task_module.miscellaneous.play_animation("Stand/Waiting/ShowMuscles_3")
        return "¡Animación 'Mostrar Músculos' reproducida con éxito!"
    except Exception as e:
        return f"Error al reproducir 'Mostrar Músculos': {e}"


@tool
def perform_bow_gesture() -> str:
    """Hace que el robot reproduzca el gesto de 'Reverencia' (por ejemplo, como agradecimiento)."""
    try:
        task_module.miscellaneous.play_animation("Stand/Gestures/BowShort_3")
        return "¡Gesto de reverencia realizado con éxito!"
    except Exception as e:
        return f"Error al realizar la reverencia: {e}"

tools = [robot_speak, change_eye_color, take_picture, get_current_time,play_dance,show_heart_gesture,make_a_call_gesture,act_like_a_zombie,drive_a_car_gesture,show_muscles,perform_bow_gesture,pose_for_picture]


class RealtimeRobotAgent:
    """Agente robot en tiempo real que responde a transcripciones de voz."""

    def __init__(self, robot_name: str = "nova"):
        """Inicializa el agente robot en tiempo real."""
        self.robot_name = robot_name
        self.robot_agent = None
        self.is_processing = False
        self.message_history = deque(maxlen=6)  # Almacena los últimos 3 pares de mensajes (humano/IA)

    def initialize(self):
        """Inicializa ROS2 y crea el agente robot con herramientas."""
        
        # Iniciar ROS2 spin en un hilo separado
        self.spin_thread = Thread(target=rclpy.spin, args=(task_module,))
        self.spin_thread.start()
        
        print(f"=== Agente Robot en Tiempo Real ({self.robot_name}) ===")
        print("Inicializando agente robot con herramientas esenciales...")
        
        # Esperar a que los servicios estén listos
        time.sleep(2)

        context = task_module.load_robot_context(robot_name=self.robot_name,language="espanol") + """
Responde de una manera agradable y sencilla y NO devuelvas formatos extraños como markdown.

IMPORTANTE: Siempre debes usar la herramienta 'robot_speak' para comunicarte verbalmente con el usuario. 
Después de hablar, puedes usar otras herramientas según sea necesario para cumplir con la solicitud del usuario.

Flujo de trabajo:
1. Usa robot_speak para responder al usuario verbalmente
2. Si la solicitud requiere acciones adicionales (cambiar color de ojos, tomar foto, bailar, etc.), usa las herramientas apropiadas
3. Si es necesario, usa robot_speak nuevamente para confirmar que las acciones se completaron

Ejemplo:
Usuario: "Cambia tus ojos a azul y baila"
1. robot_speak("¡Por supuesto! Voy a cambiar mis ojos a azul y luego bailar para ti")
2. change_eye_color("blue")
3. play_dance()

NO HAY NECESIDAD DE QUE DIGAS NADA DESPUES
4. robot_speak("¡Listo! Cambié mis ojos a azul y bailé para ti") <- NO HACER ESTO
"""

        self.llm = AzureChatOpenAI(
            azure_deployment=os.getenv("AZURE_OPENAI_CHAT_DEPLOYMENT"),
            openai_api_version=os.getenv("AZURE_OPENAI_API_VERSION"),
            api_key=os.getenv("AZURE_OPENAI_API_KEY"),
            azure_endpoint=os.getenv("AZURE_OPENAI_ENDPOINT"),
        )

        self.robot_agent = create_react_agent(
            model = self.llm,
            tools=tools,
            prompt=context
        )

        # Inicializar transcripción en tiempo real
        task_module.speech.set_transcription_mode(enabled=True, language='es')

        task_module.set_interactive_pepper_settings()
    
    def process_transcription(self, transcription_text: str):
        """Procesa la transcripción en un hilo separado para evitar bloqueos."""
        if self.is_processing:
            print("Ya se está procesando una solicitud, ignorando nueva transcripción...")
            return
            
        self.is_processing = True
        
        try:
            task_module.miscellaneous.play_animation("Stand/Waiting/Think_3")
            task_module.set_eye_color(red=255,green=255,blue=255)
            print(f'El usuario dijo: "{transcription_text}"')

            transcription_text = "Tu sistema de transcripcion de texto acaba de escuchar a la persona en frente tuyo decir esto: " + transcription_text + "Ten en cuenta que la transcripcion puede tener errores menores, has tu mejor esfuerzo por extraer informacion de lo que entiendas. Usa las herramientas necesarias para resolver lo que el usuario te pidio y siempre usa la herramienta robot_speak para hablar con el usuario, de lo contrario te quedaras callado''"

            # Deshabilitar la transcripción para evitar un bucle de retroalimentación
            print("Deshabilitando la transcripción durante la respuesta...")
            task_module.speech.set_transcription_mode(enabled=False)
            
            current_human_message = HumanMessage(content=transcription_text)
            
            agent_input = {
                "messages": list(self.message_history) + [current_human_message]
            }
            # Obtener respuesta del agente LLM
            response = self.robot_agent.invoke(agent_input) 
            
            # 3. Extraer el texto de la respuesta final de la salida del agente
            final_message = response['messages'][-1]
            final_response_text = final_message.content

            # Guardar el historial
            self.message_history.append(current_human_message)
            self.message_history.append(final_message)
            
            print(f'Mensaje final del agente: {final_response_text}')
            
            # Usar el texto final extraído para robot_speak (si el agente aún no usó la herramienta)
            if not final_message.tool_calls:
                # Si el mensaje final del agente es una respuesta de texto (no una llamada a herramienta), hacer que el robot la hable.
                # Si el agente llamó a una herramienta (como robot_speak), la ejecución de la herramienta ya ocurrió.
                # En una configuración ReAct estándar, el mensaje final de la IA es la respuesta hablada.
                #task_module.speech.say(final_response_text, animated_say=False,language_say="Spanish")
                pass
                
            # Si el mensaje final *fue* una llamada a herramienta, la ejecución de la herramienta habría ocurrido 
            # dentro del bucle del agente, y la herramienta `robot_speak` habría manejado la salida.
            
            # Pequeño retraso para asegurar que cualquier acción del robot se complete
            time.sleep(0.5)
            
        except Exception as e:
            print(f"Error al procesar la transcripción: {e}")
            # Aún intentar decir un mensaje de error
            try:
                task_module.speech.say("Lo siento, encontré un error al procesar su solicitud.",language_say="Spanish")
            except:
                pass
        
        finally:
            # Siempre volver a habilitar la transcripción
            print("Volviendo a habilitar la transcripción...")
            task_module.speech.set_transcription_mode(enabled=True, language='es')
            self.is_processing = False
    
    def on_transcription(self, msg):
        """Callback para mensajes de transcripción - no bloqueante."""
        if not msg.text.strip():  # Ignorar transcripciones vacías
            return
            
        # Comenzar el procesamiento en un hilo separado
        processing_thread = Thread(
            target=self.process_transcription, 
            args=(msg.text,)
        )
        processing_thread.start()
    
    def start_listening(self):
        """Comienza a escuchar mensajes de transcripción."""
        # Configurar la suscripción al tema de transcripción
        task_module.speech.create_subscription(
            Transcription, 
            '/microphone_node/transcription', 
            self.on_transcription, 
            10
        )
        
        print("🎤 El agente robot en tiempo real está escuchando la entrada de voz...")
        print("¡Di algo para interactuar con el robot!")
        
        # Mantener el hilo principal vivo
        try:
            while rclpy.ok():
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nApagando el agente robot en tiempo real...")
    
    def shutdown(self):
        """Apagado limpio del agente."""
        if task_module:
            task_module.destroy_node()
        rclpy.shutdown()
        if hasattr(self, 'spin_thread'):
            self.spin_thread.join()


def main():
    """Función principal para ejecutar el agente robot en tiempo real."""
    agent = RealtimeRobotAgent(robot_name="nova")
    
    try:
        # Inicializar el agente
        agent.initialize()
        # Comenzar a escuchar la entrada de voz
        agent.start_listening()
        
    except Exception as e:
        print(f"El agente falló con el error: {e}")
    
    finally:
        agent.shutdown()


if __name__ == '__main__':
    main()