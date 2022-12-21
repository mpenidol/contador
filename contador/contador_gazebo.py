import rclpy
from rclpy.node import Node
from rclpy.duration import Duration # Handles time for ROS 2
from turtlesim.msg import Pose
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from nav2_simple_commander.robot_navigator import BasicNavigator # Helper module
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from rclpy.clock import Clock
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from sklearn.cluster import DBSCAN
import time  # Time library
import numpy as np
import pandas as pd

class Gazebo_Control(Node):
    def __init__(self):
        super().__init__('gazebo_controller')
        self.get_logger().info("Controller")
        self.init_subscriber()
        self.init_variables()
        self.init_publisher1()
        self.init_publisher2()

    def init_variables(self):
        self.point = 0 #variavel dos goal points
        self.v_max = 0.2 #velocidade linear maxima do robo
        self.flag=False
        self.ranges = 0 #inicializacao do vetor do lidar
        self.positions = [] #vetor para armazenar posicoes identificadas
        self.polygon = Polygon([(0.9, 2.3),(1.1,1.8), (1.6,1.8), (2.5,0.3), (2.2,0), (2.5,-0.3), (1.6,-1.8),(-1,-1.8),(1.1,-1.9), (0.9,-2.3), (-0.8,-2.3), (-1.6,-1.8),(-2.7,0), (-1.6,1.8), (-1.0,1.8),(-0.8,2.3)])        
        #forma geometrica da estrutura do mundo
        return

    def init_subscriber(self):
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        return

    def odom_callback(self, odom: Odometry):
        #acessando as posicoes x, y e angulo do robo (radianos e grau)
        self.posicao_atual_x = odom.pose.pose.position.x
        self.posicao_atual_y = odom.pose.pose.position.y
        self.posicao_atual_theta_1 = odom.pose.pose.orientation.z
        self.posicao_atual_theta_2 = odom.pose.pose.orientation.w
        self.aux1 = 2 * ((self.posicao_atual_theta_1*self.posicao_atual_theta_2) + 0)
        self.aux2 = 1 - (2*(0+(self.posicao_atual_theta_1*self.posicao_atual_theta_1)))
        self.posicao_atual_theta = math.atan2(self.aux1,self.aux2)
        self.posicao_grau = math.degrees(self.posicao_atual_theta)
    
    def scan_callback(self, scan:LaserScan):
        #acessando o lidar do robo
        self.ranges = scan.ranges
        
    def init_publisher1(self):
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer1 = self.create_timer(0.1, self.send_cmd_vel)
        return

    #as proximas 4 funcoes sao utilizadas na modelagem controlador usado no laboratorio 04
    def euclidien_distance(self, goal_pose):
        return math.sqrt(pow((goal_pose.x - self.posicao_atual_x),2) + pow((goal_pose.y - self.posicao_atual_y),2))

    def liner_vel(self,goal_pose,constant=0.5):
        return constant*self.euclidien_distance(goal_pose)

    def steering_angle(self,goal_pose):
        return math.atan2(goal_pose.y-self.posicao_atual_y,goal_pose.x-self.posicao_atual_x)

    def angular_vel(self,goal_pose,constant=0.3):
        return constant*(self.steering_angle(goal_pose)-self.posicao_atual_theta)

    def send_cmd_vel(self):
        try:
            #as posicoes da rotina que o robo faz
            positions_x = [-1.0, 1.0,1.0,-1.0,-1.0,1.0]
            positions_y = [-1.0, -1.0,1.0,-1.0,1.0,1.0]
            theta = [0.00, 1.57, -2.21, 1.57, 0.0, 0.0]
            goal_pose=Pose()
            if self.point <= 5:
                goal_pose.x = positions_x[self.point]
                goal_pose.y = positions_y[self.point]
                goal_pose.theta = theta[self.point]
            #os comandos de velocidade
            distance_tolerance = 0.3
            angular_tolerance = 0.2
            vel_msg=Twist()
            if abs(self.steering_angle(goal_pose)-self.posicao_atual_theta)>angular_tolerance:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = self.angular_vel(goal_pose)
            else:
                vel_msg.angular.z = 0.0
                if self.point > 5:
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                    self.cmd_vel_publisher.publish(vel_msg)
                    quit()
                elif self.euclidien_distance(goal_pose)>=distance_tolerance:
                    vel_msg.linear.x = self.v_max*self.liner_vel(goal_pose)
                else:
                    self.point += 1
            self.cmd_vel_publisher.publish(vel_msg)
        except AttributeError:
            print('Aguarde')

    def init_publisher2(self):
        self.pos_publisher = self.create_publisher(Marker, '/positions', 10)
        self.timer2 = self.create_timer(0.5, self.send_pos)
        return

    def send_pos(self):
        try:
            #verificando o angulo do robo no momento em que ele ve o cilindro
            if -45 <=self.posicao_grau<45:
                self.cartesiano = []
                for i in range(len(self.ranges)):
                    if self.ranges[i] <= 1.5:
                        #fazendo um filtro de 1.5 no lidar e transformando suas distancias em cartesianas (soma-se o valor da odometrica para saber a coordenada do objeto no mapa)
                        self.cartesiano.append([self.posicao_atual_x + self.ranges[i] * math.cos(math.radians(i)), self.posicao_atual_y + self.ranges[i] * math.sin(math.radians(i))])

            elif -135<=self.posicao_grau<-45:
                self.cartesiano = []
                for i in range(len(self.ranges)):
                    if self.ranges[i] <= 1.5:
                        self.cartesiano.append([self.posicao_atual_x + self.ranges[i] * math.sin(math.radians(i) ), self.posicao_atual_y - self.ranges[i] * math.cos(math.radians(i))])  
            elif 45<=self.self.posicao_grau<135:
                self.cartesiano = []
                for i in range(len(self.ranges)):
                    if self.ranges[i] <= 1.5:
                        self.cartesiano.append([self.posicao_atual_x - self.ranges[i] * math.sin(math.radians(i) ), self.posicao_atual_y + self.ranges[i] * math.cos(math.radians(i))])  
            elif -180 <self.posicao_grau<135 or 135<=self.posicao_grau<=180:
                self.cartesiano = []
                for i in range(len(self.ranges)):
                    if self.ranges[i] <= 1.5:
                        self.cartesiano.append([self.posicao_atual_x - self.ranges[i] * math.cos(math.radians(i)), self.posicao_atual_y - self.ranges[i] * math.sin(math.radians(i))])
            
            #se apos a filtragem, existir coordenadas elas serao clusterizadas com o  dbscan
            if len(self.cartesiano)>0:
                clustering = DBSCAN(eps=0.05, min_samples=12).fit(self.cartesiano)
                IDX = clustering.labels_
                k = np.max(IDX)
                #calcula-se o numero de cluster
                medias = []
                for j in range(k+1):
                    #associa-se o cluster aos seus valores cartesinos entao é tirado uma media para saber sua coordenada central
                    mediaX = []
                    mediaY = []
                    for z in range(0, len(self.cartesiano)):
                        if clustering.labels_[z] == j:
                            mediaX.append(self.cartesiano[z][0])
                            mediaY.append(self.cartesiano[z][1])
                    if self.polygon.contains(Point(np.mean(mediaX), np.mean(mediaY))) == True: 
                        #chegado se essa cordenada central está dentro do mundo (para saber se nao eh uma parede)
                        medias.append([np.mean(mediaX),np.mean(mediaY)])
                print('Objetos sendo identificados: ', medias)
                #passando por esse processo, esse objeto clusterizado eh comparado com objetos clusterizados anteriormente, para saber se eh um novo ou nao
                if len(medias)>0:
                    if len(self.positions) == 0:
                        self.positions.append([medias[0][0], medias[0][1]])
                        
                    else:
                        for ii in range(0, len(medias)):
                            dist = []
                            dist = list(dist)
                            for jj in range(0, len(self.positions)):
                                #calcula-se a distancia euclidiana entre objetos ja identificados com potencias novos objetos para verificar isso
                                point1 = np.array((medias[ii][0], medias[ii][1]))
                                point2 = np.array((self.positions[jj][0], self.positions[jj][1]))
                                dist.append(np.linalg.norm(point1 - point2))
                            if min(dist) >0.50:
                                self.positions.append([medias[ii][0],medias[ii][1]])
                print('Objetos adicionados: ',self.positions)
                print('Objetos contabilizados:' ,len(self.positions))
                        
            else:
                print('nenhum objeto')
        except TypeError:
            pass
        except AttributeError:
            pass        

        #caso seja um novo objeto eh publicado sua posicao no mapa atraves da mensagem marker
        if len(self.positions)>0:
            marker = Marker()
            for i in range(0,len(self.positions)):
                marker.header.frame_id = "/map"
                marker.header.stamp = Clock().now().to_msg()
                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                marker.type = 3
                marker.id = i
                text = "object" + str(i)
                marker.ns = text
                # Set the scale of the marker
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.5
                # Set the color
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                # Set the pose of the marker          
                marker.pose.position.x = float(self.positions[i][0])
                marker.pose.position.y =  float(self.positions[i][1])
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                #print(marker)
                self.pos_publisher.publish(marker)
            #no fim eh retornado um csv com as posicoes e quantidade de objetos
            arr = np.asarray(self.positions)
            pd.DataFrame(arr).to_csv('~/Desktop/contagem.csv')
        return
        

def main(args=None):
    rclpy.init(args=args)
    node = Gazebo_Control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()