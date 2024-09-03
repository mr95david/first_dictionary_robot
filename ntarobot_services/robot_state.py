#!/usr/bin/env python3
# Seccion de importe de librerias
import rclpy
from rclpy.node import Node
# Seccion de importe de interfaces
from std_msgs.msg import String
from ntarobot_services.msg import StateActionC
from unique_identifier_msgs.msg import UUID
# from std_srvs.srv import SetBool
from ntarobot_services.srv import GetStatus, SetStatus
# Utilitarias
import numpy as np

# El siguiente nodo busca publicar un topico de manera que este se actualice de acuerdo al llamado
# de un servicio especifico, de esta manera se controla el estado para nuevas solicitudes deadas desde un action-servicio
class StateRobotNode(Node):
    # Constructor
    def __init__(self):
        # Herencia de clase de nodos de ros2
        super().__init__("state_robot_node")

        # Declaracion y asignacion de valor de uiid actual
        self.current_uuid_ = UUID().uuid
        # Declaracion de servicio tomado
        self.current_service = ""
        # Creacion de variables de instancia de la clase nodo
        self.current_state = "available"
        self.last_value = ""

        # Creacion de publicadores
        self.state_pub_ = self.create_publisher(
            StateActionC,
            '/ntarobot/state',
            10
        )

        # Creacion de servicios
        # Service for get current value from subs
        self.create_service(
            GetStatus,
            '/get_status',
            self.getValue
        )
        # Service for change current value
        self.create_service(
            SetStatus,
            '/change_status',
            self.changeValue
        )

        # Creacion de timer de ejecucion de valores
        self.create_timer(
            0.05,
            self.publish_state
        )

    # Funcion de publicacion de estado actual de ordenes
    def publish_state(self):
        # Declaracion de mensaje
        #msg: String = String()
        msg: StateActionC = StateActionC()

        # Asignacion de valor primero de 
        msg.state = self.current_state
        msg.uuid = self.current_uuid_
        self.state_pub_.publish(msg)

        # Validacion de estado actual de funciones de movimiento
        if self.last_value != self.current_state:
            self.get_logger().info(f"Current State robot functions: {self.current_state}")
            self.last_value = self.current_state

    # funcion para cambio de valor, servicio
    def changeValue(self, request, response):
        # Validacion de solicitud
        if request.change:
            # Asignacion de nuevo estado de ejecucion
            self.current_state = "Busy"
            # uint8_array = np.array(request.uuid, dtype=np.uint8)
            #self.get_logger().info(f"str({uint8_array})")
            self.current_uuid_ = request.uuid
        else:
            # Asignacion de nuevo estado de ejecucion
            self.current_state = "available"
            self.current_uuid_ = UUID().uuid

        response.success = True 
        response.message = f"New value of state: {self.current_state}"

        return response
    
    # Funcion para obtener el estatus actual
    # funcion para cambio de valor, servicio
    def getValue(self, request, response):
        if request.change:

            # Asignacion de valores actuales
            # Valor de estado de robot
            response.status = self.current_state
            # Uuid del action actual
            response.uuid = self.current_uuid_

        # Envio de respuesta general 
        return response
    
    # Actual_action status from nav2


    
# Funcion main para la ejecucion de nodo
def main(args=None):
    # Inicializacion de ros
    rclpy.init(args=args)
    # Creacion de objeto nodo
    state_node = StateRobotNode()
    # Validacion de ejecucion y halding error
    try:
        rclpy.spin(state_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if state_node:
            state_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

