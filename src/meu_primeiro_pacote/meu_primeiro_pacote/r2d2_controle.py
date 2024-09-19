import rclpy
import numpy
import tf_transformations
import time
import math
from math import *
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug ('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug ('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug ('Definindo o subscriber do laser: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().debug ('Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.maq_estado = 0
        self.espera(0.5)
        self.dist = 0.0
        self.dist_robo = 0.0
        self.angulo_robo = 0.0
    
    def distancia_ate_99(self):
        objective = [9,9]
        self.dist = math.dist((self.pose.position.x, self.pose.position.y), objective) 
        self.angulo_robo = math.atan2(9 - self.pose.position.x, 9 - self.pose.position.y)

    def espera(self, max_seconds):
        start = time.time()
        count = 0
        while count < max_seconds:
            count = time.time() - start            
            rclpy.spin_once(self)

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
       
    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def run(self):
        self.get_logger().debug ('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().debug ('Definindo mensagens de controde do robô.')
        self.ir_para_frente = Twist(linear=Vector3(x= 0.5,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.parar          = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))

        self.get_logger().info ('Ordenando o robô: "ir para a frente"')
        self.pub_cmd_vel.publish(self.ir_para_frente)
        rclpy.spin_once(self)

        self.get_logger().info ('Entrando no loop princial do nó.')
        while(rclpy.ok):
            
            self.pose.orientation #orientação
            self.pose.position #posição
            _, _, yaw = tf_transformations.euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]) #aqui so precisa usar o yaw
        
            rclpy.spin_once(self)

            self.get_logger().debug ('dist. laser')
            distancia_direita = numpy.array(self.laser[0:10]).mean()
            self.distancia_direita   = min((self.laser[  0: 80])) # -90 a -10 
            self.distancia_frente    = min((self.laser[ 80:100])) # -10 a  10 
            self.distancia_esquerda  = min((self.laser[100:180])) #  10 a  90 
            
            cmd = Twist()
            self.erro_ang = self.angulo_robo - yaw
            self.distancia_ate_99()
            

            if self.maq_estado == 0:
                if(abs(self.erro_ang) >= 0.06):
                    cmd.angular.z = 0.4
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().info ('virando para o 9,9')
                elif( self.dist <= 3 and abs(self.erro_ang) <= 0.06):
                    self.maq_estado = 2                
                else:
                    cmd.angular.z = 0.0
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().info ('dist=' + str(self.dist) + 'dist_robo=' + str(self.pose.position.x)+ str(self.pose.position.y))
                    #print(self.dist, self.distancia_frente)
                    self.maq_estado = 1
        
            elif self.maq_estado == 1: #sem caixas a vista, reto
                if(self.distancia_frente > self.distancia_direita and self.distancia_frente > self.distancia_esquerda and self.distancia_frente > 1):
                    self.get_logger().info ('nada na frente')
                    self.maq_estado = 2
                elif(self.distancia_esquerda > self.distancia_direita and self.distancia_esquerda > self.distancia_frente ):
                    cmd.angular.z = 0.5
                    self.get_logger().info ('caixa na frente, virando a esquerda')
                    self.pub_cmd_vel.publish(cmd)
                elif(self.distancia_direita > self.distancia_frente and self.distancia_direita > self.distancia_esquerda ):
                    cmd.angular.z = -0.5
                    self.get_logger().info ('ainda tem caixa na frente, virando a direita')
                    self.pub_cmd_vel.publish(cmd)
                else:
                    cmd.angular.z = 0.5
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().info ('aaaaa, vou ficar tonto de tanto girar')

            elif self.maq_estado == 2: #com caixas :o
                    cmd.linear.x = 0.5
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().debug ("Distância para o obstáculo" + str(self.distancia_frente))
                
                    if(self.dist <= 3 and self.dist >=1 and abs(self.erro_ang) <= 0.06):
                        cmd.linear.x = 0.5
                        self.pub_cmd_vel.publish(cmd)
                        self.get_logger().info ('andandooo')
                    elif (self.distancia_frente < self.distancia_direita and self.distancia_frente < self.distancia_esquerda or self.distancia_frente < 1):
                        self.maq_estado = 0
                    if(self.dist <= 0.8):
                        cmd.angular.z = 0.0
                        cmd.linear.x = 0.0
                        self.pub_cmd_vel.publish(cmd)
                        self.get_logger().info ('cheguei no 9,9')

        self.get_logger().info ('Ordenando o robô: "parar"')
        self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)


    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')

# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()  