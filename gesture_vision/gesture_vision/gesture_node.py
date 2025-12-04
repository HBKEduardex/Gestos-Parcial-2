import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import mediapipe as mp
import cv2
import math

class GestureNode(Node):
    def __init__(self):
        super().__init__('gesture_node')

        self.bridge = CvBridge()

        # Suscribirse al video del Kinect (rosbag)
        self.create_subscription(
            Image,
            '/kinect/image_raw',
            self.image_callback,
            10
        )

        # Publicar comando de gesto
        self.pub = self.create_publisher(String, '/gesture_command', 10)

        # MediaPipe
        self.mp_pose = mp.solutions.pose.Pose()

        self.get_logger().info("GestureNode listo. Leyendo /kinect/image_raw y publicando /gesture_command.")

    # ===============================
    # 1. Calcular ángulo del brazo
    # ===============================
    def compute_arm_angle(self, shoulder, wrist):
        dx = wrist.x - shoulder.x
        dy = shoulder.y - wrist.y   # invertido
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    # ===============================
    # 2. Clasificación SEGÚN TUS FOTOS
    # ===============================
    def classify_gesture(self, angle_r, angle_l):
        up = 70          # arriba
        down = -70       # abajo
        horiz_low = -15  # horizontal mínimo
        horiz_high = 15  # horizontal máximo

        # ADELANTE: ambos brazos arriba
        if angle_r > up and angle_l > up:
            return "forward"

        # ATRÁS: ambos brazos abajo
        if angle_r < down and angle_l < down:
            return "backward"

        # IZQUIERDA: brazo izquierdo horizontal, brazo derecho abajo
        if 110 <= angle_l  and angle_r < -170:
            return "left"

        # DERECHA: brazo derecho horizontal, brazo izquierdo abajo
        if  -30 <= angle_r <= -10 and 0< angle_l < 10:
            return "right"

        # STOP: ambos horizontales
        if horiz_low <= angle_r <= horiz_high and horiz_low <= angle_l <= horiz_high:
            return "stop"

        # Cualquier otra cosa
        return "stop"

    # ===============================
    # 3. Procesar imagen y publicar gesto
    # ===============================
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = self.mp_pose.process(rgb)

        if not result.pose_landmarks:
            self.pub.publish(String(data="stop"))
            return

        lm = result.pose_landmarks.landmark

        r_sh = lm[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]
        r_wr = lm[mp.solutions.pose.PoseLandmark.RIGHT_WRIST]
        l_sh = lm[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
        l_wr = lm[mp.solutions.pose.PoseLandmark.LEFT_WRIST]

        # Calcular ángulos
        angle_r = self.compute_arm_angle(r_sh, r_wr)
        angle_l = self.compute_arm_angle(l_sh, l_wr)

        # Obtener gesto
        cmd = self.classify_gesture(angle_r, angle_l)

        # Publicar
        self.pub.publish(String(data=cmd))

        # Log
        self.get_logger().info(
            f"Ángulos -> R: {angle_r:.1f}°, L: {angle_l:.1f}°  → Gesto: {cmd}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
