import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi
import time


class MoveRover(Node):
    def __init__(self):
        super().__init__('move_rover')
        # Initialisation du noeud ROS et des publishers/subscribers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Variables de suivi de la position et de l'état
        self.current_pose = None  # Position actuelle
        self.start_pose = None  # Position de départ pour chaque segment
        self.phase = "move_1_5"  # Phase initiale : avancer de 1.5m
        self.linear_speed = 1.5  # Vitesse linéaire (m/s)
        self.angular_speed = 0.2  # Vitesse angulaire (rad/s)
        self.angle_tolerance = 0.01  # Tolérance pour l'alignement angulaire (rad)
        self.sides_completed = 0  # Compteur de segments complétés
        self.first_square = True  # Indique si on est encore au premier carré
        self.side_lengths_initial = [1.5, 1.5, 3.0, 3.0, 3.0, 3.0]  # Segments pour atteindre le périmètre
        self.side_lengths_repeat = [3.0, 3.0, 3.0, 3.0]  # Segments pour les carrés suivants
        self.current_side_lengths = self.side_lengths_initial  # Liste active de segments
        self.current_side_index = 0  # Index du segment en cours
        self.initial_yaw = None  # Angle initial avant une rotation
        self.target_yaw = None  # Angle cible pour une rotation

    def odom_callback(self, msg):
        """
        Callback pour recevoir les données de position /odom.
        Met à jour la position actuelle du rover et gère les phases de déplacement.
        """
        self.current_pose = msg.pose.pose

        if self.start_pose is None:
            self.start_pose = self.current_pose
            return

        # Phase de déplacement linéaire
        if "move" in self.phase:
            if self.distance_traveled() < self.current_side_lengths[self.current_side_index]:
                self.move_forward()  # Continuer à avancer
            else:
                self.stop()  # Arrêter le rover une fois la distance atteinte
                time.sleep(0.1)  # Pause pour stabiliser le mouvement
                self.start_pose = self.current_pose  # Réinitialiser la position de départ
                self.initial_yaw = None  # Réinitialiser l'angle initial
                self.target_yaw = None  # Réinitialiser l'angle cible
                self.phase = f"turn_{self.current_side_index}"  # Passer à la phase de rotation
                self.get_logger().info(f"Segment {self.current_side_index + 1} complété. Rotation en cours.")

        # Phase de rotation
        elif "turn" in self.phase:
            if self.rotate_by_angle(pi / 2):  # Tourner de 90 degrés (pi/2 radians)
                self.start_pose = self.current_pose  # Réinitialiser la position de départ
                time.sleep(0.1)  # Pause pour stabiliser après la rotation
                self.current_side_index += 1  # Passer au segment suivant
                if self.current_side_index < len(self.current_side_lengths):
                    self.phase = f"move_{self.current_side_index}"  # Passer à la phase suivante
                    self.get_logger().info(f"Rotation complétée. Segment {self.current_side_index + 1} démarré.")
                else:
                    # Réinitialiser pour un nouveau carré
                    self.current_side_index = 0
                    self.first_square = False  # Le premier carré est complété
                    self.current_side_lengths = self.side_lengths_repeat  # Passer à la séquence répétée
                    self.phase = f"move_{self.current_side_index}"
                    self.get_logger().info("Carré terminé. Nouveau carré en cours.")

    def rotate_by_angle(self, target_angle):
        """
        Effectue une rotation précise en fonction d'un angle cible.
        :param target_angle: L'angle à atteindre (en radians).
        :return: True si la rotation est terminée, False sinon.
        """
        current_yaw = self.get_yaw(self.current_pose.orientation)

        # Définir l'angle initial et cible une seule fois
        if self.initial_yaw is None:
            self.initial_yaw = current_yaw
            self.target_yaw = self.initial_yaw + target_angle
            self.target_yaw = (self.target_yaw + pi) % (2 * pi) - pi  # Normaliser entre -pi et pi

        # Calculer l'erreur angulaire par rapport à la cible
        error = self.target_yaw - current_yaw
        error = (error + pi) % (2 * pi) - pi  # Normaliser entre -pi et pi

        if abs(error) > self.angle_tolerance:  # Si l'erreur est supérieure à la tolérance
            twist = Twist()
            twist.angular.z = self.angular_speed if error > 0 else -self.angular_speed
            self.publisher.publish(twist)  # Publier la commande de rotation
            return False  # Continuer la rotation
        else:
            self.stop()  # Arrêter la rotation
            time.sleep(0.1)  # Pause pour stabiliser
            self.initial_yaw = None  # Réinitialiser pour la prochaine rotation
            self.target_yaw = None
            return True  # Rotation terminée

    def distance_traveled(self):
        """
        Calcule la distance parcourue depuis la position de départ.
        :return: Distance parcourue (en mètres).
        """
        dx = self.current_pose.position.x - self.start_pose.position.x
        dy = self.current_pose.position.y - self.start_pose.position.y
        return sqrt(dx**2 + dy**2)

    def get_yaw(self, orientation):
        """
        Convertit un quaternion en angle de lacet (yaw).
        :param orientation: Orientation au format quaternion.
        :return: Angle de lacet (en radians).
        """
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return atan2(siny_cosp, cosy_cosp)

    def move_forward(self):
        """
        Publie une commande pour faire avancer le rover en ligne droite.
        """
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.publisher.publish(twist)

    def stop(self):
        """
        Publie une commande pour arrêter tous les mouvements du rover.
        """
        twist = Twist()
        self.publisher.publish(twist)


def main(args=None):
    """
    Point d'entrée principal du programme.
    Initialise ROS 2, lance le nœud et attend les commandes.
    """
    rclpy.init(args=args)
    node = MoveRover()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
