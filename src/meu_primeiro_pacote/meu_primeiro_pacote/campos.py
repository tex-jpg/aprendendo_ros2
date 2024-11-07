import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, Twist

class Robot:
    def __init__(self, position, velocity):
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)

    def update(self, delta_time):
        self.position += self.velocity * delta_time

def attractive_force(robot_position, goal_position, alpha=1):
    return alpha * (goal_position - robot_position)

def repulsive_force(robot_position, obstacle_position, beta=1, gamma=1):
    distance = np.linalg.norm(robot_position - obstacle_position)
    if distance < 40:
        beta = beta * np.interp(distance, [0, 40], [150.0, 1.0])
        gamma = gamma * np.interp(distance, [0, 40], [280.0, 1.0])
    direction = robot_position - obstacle_position
    repulsive_component = beta * direction / distance**gamma
    tangential_component = gamma * (1.0 / distance) * np.array([-direction[1], direction[0]])
    tangential_weight = max(1.0 - np.exp(-distance**2 / (2 * (distance/2)**2)), 1.0)
    total_force = repulsive_component + tangential_weight * tangential_component
    return total_force

def calculate_total_force(robot_position, goal_position, obstacles):
    total_force = attractive_force(robot_position, goal_position)
    for obstacle in obstacles:
        total_force += repulsive_force(robot_position, obstacle)
    return total_force

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
