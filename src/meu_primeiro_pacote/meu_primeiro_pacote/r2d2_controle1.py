import rclpy 
import numpy 
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug('Definindo o subscriber do laser: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().debug('Definindo o publisher de controle do robo: "/cmd_vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def run(self):
        self.get_logger().debug('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().debug('Definindo mensagens de controle do robô.')
        self.ir_para_frente = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.get_logger().info('Ordenando o robô: "ir para a frente"')
        self.pub_cmd_vel.publish(self.ir_para_frente)

        rclpy.spin_once(self)

        self.get_logger().info('Entrando no loop principal do nó.')
        integral = 0.0
        # Inicia o loop principal para controle do robô.

        while(rclpy.ok):
            rclpy.spin_once(self)
            # Continua processando mensagens enquanto o ROS estiver ativo.

            distancia_objetivo = 2
            distancia_direita = numpy.mean(self.laser[0:10])
            distancia_frente = numpy.mean(self.laser[80:100])
            p_gain = 1.95
            i_gain = 0.00
            d_gain = 0.01
            # Coleta dados do sensor de laser e define variáveis de controle PID.

            def parede():
                integral = 0.0
                error = distancia_objetivo - distancia_direita
                integral = integral + error
                old_error = error
                dif_erro = error - old_error
                power = p_gain * error + i_gain * integral + d_gain * dif_erro
                cmd = Twist()
                cmd.linear.x = 0.5
                cmd.angular.z = power
                self.pub_cmd_vel.publish(cmd)
                # Função para seguir a parede, ajustando o movimento com base no erro calculado pelo controle PID.

            def virar():
                integral = 0.0
                error = distancia_objetivo - distancia_direita
                integral = integral + error
                old_error = error
                dif_erro = error - old_error
                power = p_gain * error + i_gain * integral + d_gain * dif_erro
                cmd = Twist()
                cmd.linear.x = power
                cmd.angular.z = 0.5
                self.pub_cmd_vel.publish(cmd)
                # Função para virar o robô quando houver um obstáculo na frente.

            if (distancia_frente < distancia_objetivo):
                virar()
                self.get_logger().info('vira')
                # Se houver um obstáculo na frente, chama a função de virar.
            else:
                parede()
                # Caso contrário, segue a parede.

        self.get_logger().info('Ordenando o robô: "parar"')
        self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)
        # Para o robô no final do loop.

    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')
        # Mensagem de log quando o nó é destruído.

def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    # Função principal que inicializa o ROS, executa o nó e lida com o encerramento do sistema.
if __name__ == '__main__':
    main()