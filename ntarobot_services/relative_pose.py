#!/usr/bin/env python3
# Seccion de importe de librerias
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Importe de librerias de interfaces
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from ntarobot_services.srv import RelativeMove
from nav2_msgs.action import NavigateToPose

# Librerias utilitarias
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math

# Clase de nodo de ejecucion
class RelativeMovePose(Node):
    def __init__(self):
        super().__init__("service_relative_move")

        # Inicializacion de variables
        self.actual_pose = PoseWithCovarianceStamped()

        # Inicializacion de subscribers
        self.pose_subs = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )
        # Validacion de ejecucion de subscriptor
        self.pose_subs

        # Creacion de servicio
        self.relative_move_service = self.create_service(
            RelativeMove,
            "relative_move",
            self.serviceCallback
        )

        # Creacion de action de ejecucion de movimiento
        self.action_nav = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        # Validacion de definicion de servicio de move pose
        self.get_logger().info("Relative Move service in action")

    # Seccion de funciones de callback
    def pose_callback(self, msg: PoseWithCovarianceStamped):

        # Asignacion a valor de variable de instancia
        self.actual_pose = msg

    # Callback de solicitud de servicio
    def serviceCallback(self, request, response):
        # Validacion de inicio de ejecucion de servicio
        self.get_logger().info(f"Request of move relative is linear: {request.linear} - angular: {request.angular}")

        # Posicion inicial de robot segun lectura
        x_position_in = self.actual_pose.pose.pose.position.x
        y_position_in = self.actual_pose.pose.pose.position.y

        # Se ejecuta el cambio de quaternion to euler
        q_ = [
            self.actual_pose.pose.pose.orientation.x,
            self.actual_pose.pose.pose.orientation.y,
            self.actual_pose.pose.pose.orientation.z,
            self.actual_pose.pose.pose.orientation.w
        ]

        # Changes values
        _, _, z_orientation_in = euler_from_quaternion(q_)

        # Uso de funcion para definicion de posicion final
        pose_final :PoseStamped= self.get_finalPose(
            [x_position_in, y_position_in, z_orientation_in],
            [request.linear, request.angular]
        )

        # Definicion de pose objetivo
        goal_msg = NavigateToPose.Goal()

        # Asignacion de valor a pose goal
        goal_msg.pose = pose_final

        # Seccion de ejecucion de server y action
        self.action_nav.wait_for_server()

        # Envio de objetivo
        self.send_goal_future = self.action_nav.send_goal_async(goal_msg)
        # Validacion de cumplimiento de objetivo
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        return response
    
    # Funcion que calcula la posicion final a partir de datos relativos
    def get_finalPose(
        self,
        pose_values: list,
        diference_values: list
    ) -> PoseStamped:
        
        # Definicion de mensaje de pose final
        final_pose = PoseStamped()

        # Calculo de diferencia angular
        theta_final = pose_values[-1] + diference_values[-1]

        # Calculo de x y final
        x_final = pose_values[0] + (diference_values[0]*math.cos(theta_final))
        y_final = pose_values[1] + (diference_values[0]*math.sin(theta_final))

        # Asignacion de valores de header - default
        final_pose.header.frame_id = "map"
        final_pose.header.stamp = self.get_clock().now().to_msg()

        # Asignacion de valores de cada posicion
        final_pose.pose.position.x = x_final
        final_pose.pose.position.y = y_final

        # Conversion de angulo a quaternion
        q = quaternion_from_euler(0, 0, theta_final)
        # Asignacion de valroes de orientacion
        final_pose.pose.orientation.x = q[0]
        final_pose.pose.orientation.y = q[1]
        final_pose.pose.orientation.z = q[2]
        final_pose.pose.orientation.w = q[3]

        # Respuesta final
        return final_pose

    # Seccion de callbacks de action
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

def main():
    rclpy.init()

    move_pose = RelativeMovePose()
    rclpy.spin(move_pose)
    
    move_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()