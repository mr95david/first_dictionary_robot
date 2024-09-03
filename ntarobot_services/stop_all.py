#!/usr/bin/env python3
# Seccion de importe de librerias
import rclpy
from rclpy.node import Node
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool
from unique_identifier_msgs.msg import UUID
from action_msgs.srv import CancelGoal
from ntarobot_services.srv import GetStatus
# Utilitarias import 
import time

# Creacion de clase para ejecucion de nodo
class StopRobotSuccess(Node):
    # Constructor
    def __init__(self):
        # Herencia de clase de nodos de ros2
        super().__init__("state_robot_node")

        # Declaracion de variables de uso
        self.list_services = [
            "move_to_pose",
            "relative_move"
        ]

        # Creacion de cliente de servicio
        self.service_client = None

        # Creacion de servicio para detencion de nodo
        self.srv_cancel_ = self.create_service(
            SetBool,
            "cancel_all",
            self.cancel_navigation_callback
        )

    # BORRA ESTA SECCION
    # Nueva función de callback para el servicio de cancelación
    def cancel_navigation_callback(self, request, response):
        # Manejador de cancelación del objetivo
        for service in self.list_services:
            try:
                # Definicion de cliente de ejecucion
                self.service_client = self.create_client(
                    SetBool,
                    service+"/cancel_goal"
                )
                # Espera de validacion de cliente
                while not self.service_client.wait_for_service(timeout_sec = 0.5):
                    # Validacion de conexion
                    self.get_logger().info('Service not available, waiting...')

                request_ = SetBool.Request()
                request_.data = True
                fut_statusService = self.service_client.call_async(request_)
                time.sleep(0.01)
                fut_statusService.add_done_callback(self.cancel_response_callback)


            except Exception as e:
                raise TypeError(f"Error intentando ejecutar el cliente de servicio. {e}")
            
            finally:
                self.destroy_client(self.service_client)
                self.service_client = None
        
        response.success = True
        response.message = "Objectives cancel success"
        
        return response

    # Nueva función de callback para la respuesta de cancelación
    def cancel_response_callback(self, future):
        cancel_response = future.result()
        self.get_logger().info('El objetivo fue cancelado con éxito.')

        return 

# Funcion main para la ejecucion de nodo
def main(args=None):
    # Inicializacion de ros
    rclpy.init(args=args)
    # Creacion de objeto nodo
    robot_stop = StopRobotSuccess()
    # Validacion de ejecucion y halding error
    try:
        rclpy.spin(robot_stop)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if robot_stop:
            robot_stop.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
