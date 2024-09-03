#!/usr/bin/env python3
# Seccion de importe de librerias
# Librerias de ros2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, CancelResponse
# import de interfaces
from ntarobot_services.srv import MoveToPose
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID
from ntarobot_services.msg import StateActionC
from ntarobot_services.srv import SetStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from std_srvs.srv import SetBool

# Utilitarias
from time import sleep 

# Seccion de clase de ejecucion de nodo
class MoveToPoseService(Node):
    # Constructor
    def __init__(self):
        super().__init__("move_pose_service")
        # Creacion de cliente de servicio
        self.client_ = self.create_client(
            SetStatus,
            '/change_status'
        )

        # Asignacion de cliente
        while not self.client_.wait_for_service(timeout_sec = 1.0):
            # Validacion de conexion
            self.get_logger().info('Service not available, waiting...')

        # asignar request
        self.request_ = SetStatus.Request()

        # Inicializacion de variables de instancia
        # Inicializacion de Servicio
        self.srv_pose_ = self.create_service(
            MoveToPose,
            "move_to_pose",
            self.serviceCallback
        )
        # Inicializacion de Action
        self.action_nav = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )
        
        # Temporal: NO FUNCIONA BORRAR
        self.srv_cancel_ = self.create_service(
            SetBool,
            "move_to_pose/cancel_goal",
            self.cancel_navigation_callback
        )
        # # BORRAR HASTA ACA

        # Validacion de definicion de servicio de move pose
        self.get_logger().info("Service for move to pose init")
        
    # Funcion de callback de servicio
    def serviceCallback(self, request, response):
        # Validacion de ejecucion de servidor
        self.get_logger().info(f"Request of navigate to pose x: {request.x} - y: {request.y} - theta: {request.theta}")

        # Define pose msg
        msg_pose = self.def_pose(
            float(request.x), 
            float(request.y), 
            float(request.theta)
        )

        # Seccion de creacion de mensaje de ejecucion
        goal_msg = NavigateToPose.Goal()

        # Asignacion de valor a pose goal
        goal_msg.pose = msg_pose

        # Seccion de ejecucion de server y action
        self.action_nav.wait_for_server()

        # Envio de objetivo
        self.send_goal_future = self.action_nav.send_goal_async(goal_msg)
        # Validacion de cumplimiento de objetivo
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        return response
    
    # Funcion de validacion de cumplimiento de objetivo
    def goal_response_callback(self, future):
        # Manejador de resultado futuro
        goal_handle = future.result()

        # Conficional para deteccion de recibimiento de objetivo
        if not goal_handle.accepted:
            self.get_logger().info('El goal fue rechazado')
            return
        self.get_logger().info('El goal fue aceptado')

        # Visualizacion de id de ejecucion
        # self.get_logger().warn("TIPO DE DATO PUBLICADO")
        # self.get_logger().warn(str(goal_handle.goal_id))

        # Designacion de funcionamiento de servicio de cambio de status 
        # Designacion de valor enviado, true para ocupar el robot, false para desocupar
        self.request_.change = True
        #self.get_logger().info(str(goal_handle.goal_id.uuid))
        self.request_.uuid = goal_handle.goal_id.uuid
        sleep(0.01)
        fut_statusService = self.client_.call_async(self.request_)
        sleep(0.01)
        fut_statusService.add_done_callback(self.servie_response_callback)

        # Recibimiento de informacion de la ccion
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    # Funcion de validacion visual de estado de action
    def get_result_callback(self, future):
        result = future.result()
        
        if result is None:
            self.get_logger().error('Error al obtener el resultado de la acción.')
            return

        # Verifica el estado final de la acción
        if result.status == 4:  # El número 4 representa el estado "SUCCEEDED"
            # Designacion de valor enviado, true para ocupar el robot, false para desocupar
            self.request_.change = False
            self.request_.uuid = UUID().uuid
            fut_statusService = self.client_.call_async(self.request_)
            sleep(0.01)
            fut_statusService.add_done_callback(self.servie_response_callback)
            # ejecucion de servicio de cambio de valor
            
            # Validacion de tarea completada
            self.get_logger().info('El robot alcanzó la pose deseada con éxito.')
        else:
            # Designacion de valor enviado, true para ocupar el robot, false para desocupar
            self.request_.change = False
            self.request_.uuid = UUID().uuid
            fut_statusService = self.client_.call_async(self.request_)
            sleep(0.01)
            fut_statusService.add_done_callback(self.servie_response_callback)
            # ejecucion de servicio de cambio de valor

            # Validacion de tarea no completada
            self.get_logger().warn(f'El robot no alcanzó la pose deseada. Estado: {result.status}')

    # Funcion para definicion de mensaje de posicion
    def def_pose(self, x:float, y:float, theta:float) -> PoseStamped:
        msg = PoseStamped()

        # Asignacion de valores de header - default
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Valores de posicion
        msg.pose.position.x = x
        msg.pose.position.y = y

        # Transformacion de angulo to quaternion
        q = quaternion_from_euler(0, 0, theta)
        # Valores de orientacion
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        return msg

    # Creacion de funcion de respuesta de servicio
    def servie_response_callback(self, fut):
        # Validacion de estado de ejecucion
        try:
            response = fut.result()
            # if response.success:
            #     self.get_logger().info('Servicio llamado con exito')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    # # BORRA ESTA SECCION
    # Nueva función de callback para el servicio de cancelación
    def cancel_navigation_callback(self, request, response):
        # Manejador de cancelación del objetivo
        if hasattr(self, 'send_goal_future'):
            goal_handle = self.send_goal_future.result()
            if goal_handle is not None:
                self.get_logger().info('Cancelando el objetivo...')
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_response_callback)
                response.success = True
                response.message = "Objetivo cancelado."
            else:
                self.get_logger().warn('No hay un objetivo activo para cancelar.')
                response.success = False
                response.message = "No hay un objetivo activo."
        else:
            self.get_logger().warn('No hay un objetivo para cancelar.')
            response.success = False
            response.message = "No hay un objetivo activo."
        
        return response

    # Nueva función de callback para la respuesta de cancelación
    def cancel_response_callback(self, future):
        cancel_response = future.result()
        self.get_logger().info(str(cancel_response.return_code))
        # Validacion de estado de cancelacion final
        if cancel_response.return_code is not None:
            # Validacion de tipo de ejecucion
            if cancel_response.return_code == 0:
                self.get_logger().info('El objetivo fue cancelado con éxito.')
            # Validacion en caso de no encontrar el identificador especifico
            elif cancel_response.return_code == 1:
                self.get_logger().warn('No existen objetivos por cancelar.')
            elif cancel_response.return_code == 2:
                self.get_logger().warn('El ID del objetivo no fue encontrado')
            # Error general
            elif cancel_response.return_code == 3:
                self.get_logger().error('El servicio de cancelacion no pudo ser ejecutado.')


def main():
    rclpy.init()

    move_pose = MoveToPoseService()
    rclpy.spin(move_pose)
    
    # move_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()