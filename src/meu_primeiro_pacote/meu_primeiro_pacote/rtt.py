import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, Twist
class Robot:
    def __init__(self, position=[0, 0], velocity=[0, 0]):
        self.position = np.array(position)
        self.velocity = np.array(velocity)

    def update(self, dt):
        self.position += self.velocity * dt
class RRTRobotNavigationNode(Node):
    def __init__(self):
        super().__init__('robot_navigation_rrt')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.robot_position = (300, 25)  # Posição inicial
        self.goal = (80, 325)  # Posição do objetivo
        self.max_interactions = 1000  # Máximo de iterações
        self.fator_crescimento = 10  # Passo do RRT
        self.mapa_binario = np.ones((350, 350))  # Mapa binário (obstáculos são 1, espaço livre é 0)
        self.mapa_binario[80][325] = 0  # Marca o objetivo no mapa
        self.mapa_binario[300][25] = 0  # Marca a posição inicial no mapa
        self.timer = self.create_timer(0.1, self.update_callback)
        self.nos_explorados = [self.robot_position]
        self.caminhos_explorados = []
        self.pais_nos, self.filhos_nos = [], []

    def update_callback(self):
        # Executar uma iteração do RRT
        ponto_aleatorio = (random.randint(0, 350), random.randint(0, 350))
        no_mais_proximo = self.encontrar_no_mais_proximo(self.nos_explorados, ponto_aleatorio)
        novo_no = self.gerar_novo_no(no_mais_proximo, ponto_aleatorio)
        
        if self.mapa_binario[novo_no[0]][novo_no[1]] == 0:  # Verifica se o ponto está livre (não colide com obstáculos)
            self.nos_explorados.append(novo_no)
            self.caminhos_explorados.append((no_mais_proximo, novo_no))
            self.pais_nos.append(no_mais_proximo)
            self.filhos_nos.append(novo_no)
            
            # Verifica se o robô chegou ao objetivo
            if np.linalg.norm(np.array(novo_no) - np.array(self.goal)) < self.fator_crescimento:
                self.encontrar_caminho_final(novo_no)
                self.visualizar_caminho()
                self.stop_navigation()

    def encontrar_no_mais_proximo(self, nos_explorados, ponto_aleatorio):
        return min(nos_explorados, key=lambda no: math.dist(no, ponto_aleatorio))

    def gerar_novo_no(self, no_mais_proximo, ponto_aleatorio):
        vetor_direcao = (ponto_aleatorio[0] - no_mais_proximo[0], ponto_aleatorio[1] - no_mais_proximo[1])
        modulo_vetor = math.sqrt(vetor_direcao[0]**2 + vetor_direcao[1]**2)
        if modulo_vetor < self.fator_crescimento:
            return ponto_aleatorio
        fator = self.fator_crescimento / modulo_vetor
        return (int(no_mais_proximo[0] + vetor_direcao[0] * fator), int(no_mais_proximo[1] + vetor_direcao[1] * fator))

    def encontrar_caminho_final(self, no_filho):
        caminho_final = []
        while no_filho != self.robot_position:
            indice = self.filhos_nos.index(no_filho)
            no_pai = self.pais_nos[indice]
            caminho_final.append((no_pai, no_filho))
            no_filho = no_pai
        self.caminho_final = caminho_final

    def visualizar_caminho(self):
        mapa_colorido = np.ones_like(self.mapa_binario) * 255  # Cria uma cópia da imagem do mapa
        for (ponto1, ponto2) in self.caminhos_explorados:
            cv2.line(mapa_colorido, ponto1[::-1], ponto2[::-1], (255, 0, 0), 1)  # Caminho explorado em vermelho

        # Desenha o caminho final em verde
        for (ponto1, ponto2) in self.caminho_final:
            cv2.line(mapa_colorido, ponto1[::-1], ponto2[::-1], (0, 255, 0), 2)  # Caminho final em verde

        # Exibe o mapa com o caminho
        plt.imshow(mapa_colorido)
        plt.title('Caminho do robô até o objetivo')
        plt.show()

    def stop_navigation(self):
        self.robot_position = self.goal
        print("Objetivo alcançado!")


class RobotNavigationNode(Node):
    def __init__(self):
        super().__init__('robot_navigation')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.robot = Robot(position=[0, 0], velocity=[0, 0])
        self.goal = np.array([50, 50])
        self.obstacles = np.array([[25, 0], [30, 0], [400, 400]])
        self.max_speed = 10.0
        self.max_acceleration = 1.0
        self.timer = self.create_timer(0.1, self.update_callback)

    def update_callback(self):
        force = calculate_total_force(self.robot.position, self.goal, self.obstacles)
        acceleration = force - self.robot.velocity
        acceleration_magnitude = np.linalg.norm(acceleration)
        if acceleration_magnitude > self.max_acceleration:
            acceleration = acceleration * (self.max_acceleration / acceleration_magnitude)
        
        self.robot.velocity += acceleration * 0.1
        speed = np.linalg.norm(self.robot.velocity)
        if speed > self.max_speed:
            self.robot.velocity = self.robot.velocity * (self.max_speed / speed)

        self.robot.update(0.1)

        # Create and publish Twist message
        vel_msg = Twist()
        vel_msg.linear.x = float(self.robot.velocity[0])/10
        vel_msg.linear.y = float(self.robot.velocity[1])/10
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(vel_msg)

        # Debugging Information
        print(f"Position: {self.robot.position}, Velocity: {self.robot.velocity}, Speed: {speed}")

        if np.linalg.norm(self.robot.position - self.goal) < 5 and speed < 1.0:
            self.robot.velocity = np.array([0.0, 0.0])
            

def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
  