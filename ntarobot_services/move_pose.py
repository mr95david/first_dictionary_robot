#!/usr/bin/env python3
# Seccion de importe de librerias
# Librerias de ros2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# import de interfaces
from ntarobot_services.srv import MoveToPose
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

# Seccion de clase de ejecucion de nodo
class MoveToPoseService(Node):
    # Constructor
    def __init__(self):
        super().__init__("move_pose_service")

        # Inicializacion de variables de instancia
        # Inicializacion de Action
        self.action_nav = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )
        # Inicializacion de Servicio
        self.srv_pose_ = self.create_service(
            MoveToPose,
            "move_to_pose",
            self.serviceCallback
        )

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
            self.get_logger().info('El robot alcanzó la pose deseada con éxito.')
        else:
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

def main():
    rclpy.init()

    move_pose = MoveToPoseService()
    rclpy.spin(move_pose)
    
    # move_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()