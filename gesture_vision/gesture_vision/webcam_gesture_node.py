import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import math


class WebcamGestureNode(Node):
    def __init__(self):
        super().__init__('webcam_gesture_node')

        # Publicador de comando
        self.pub = self.create_publisher(String, '/gesture_command', 10)

        # Cámara web (0 = default)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("ERROR: No se pudo abrir la cámara web")

        # MediaPipe
        self.mp_pose = mp.solutions.pose.Pose()

        # Timer para leer frames (~20 FPS)
        self.timer = self.create_timer(0.05, self.process_frame)

        self.get_logger().info("WebcamGestureNode listo -> usando webcam y publicando /gesture_command")

    # ----------------------------------------
    # Calcular ángulo del brazo
    # ----------------------------------------
    def compute_arm_angle(self, shoulder, wrist):
        dx = wrist.x - shoulder.x
        dy = shoulder.y - wrist.y
        angle = math.degrees(math.atan2(dy, dx))
        return angle

    # ----------------------------------------
    # CLASIFICACIÓN USANDO TUS CONDICIONES EXACTAS
    # ----------------------------------------
    def classify_gesture(self, angle_r, angle_l):

        up = 70          # arriba
        down = -70       # abajo
        horiz_low = -15  # horizontal mínimo
        horiz_high = 15  # horizontal máximo

        # ADELANTE
        if 75 < angle_r < 100 and 75 < angle_l < 100:
            return "forward"

        # ATRÁS
        if -75 > angle_r > -100 and -75 > angle_l > -100:
            return "backward"

        # IZQUIERDA
        if angle_l <= -110 and angle_r > 30:
            return "left"

        # DERECHA
        if angle_r <= 60 and -5 < angle_l < 10:
            return "right"

        # STOP
        if horiz_low <= angle_r <= horiz_high and horiz_low <= angle_l <= horiz_high:
            return "stop"

        return "stop"

    # ----------------------------------------
    # Procesar frame de la webcam
    # ----------------------------------------
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.mp_pose.process(rgb)

        if not result.pose_landmarks:
            self.pub.publish(String(data="stop"))
            cv2.imshow("Webcam Gesture", frame)
            cv2.waitKey(1)
            return

        lm = result.pose_landmarks.landmark

        r_sh = lm[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]
        r_wr = lm[mp.solutions.pose.PoseLandmark.RIGHT_WRIST]
        l_sh = lm[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
        l_wr = lm[mp.solutions.pose.PoseLandmark.LEFT_WRIST]

        angle_r = self.compute_arm_angle(r_sh, r_wr)
        angle_l = self.compute_arm_angle(l_sh, l_wr)

        cmd = self.classify_gesture(angle_r, angle_l)

        self.pub.publish(String(data=cmd))

        self.get_logger().info(
            f"[WEB] Ángulos -> R: {angle_r:.1f}°, L: {angle_l:.1f}°  → {cmd}"
        )

        cv2.imshow("Webcam Gesture", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamGestureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
