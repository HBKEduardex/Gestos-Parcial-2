import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import cv2


class DepthZonesNode(Node):
    def __init__(self):
        super().__init__('depth_zones_node')

        self.bridge = CvBridge()

        # Suscriptor a imagen de profundidad del Kinect (en mm, 16UC1)
        self.create_subscription(
            Image,
            '/kinect/depth/image_raw',
            self.depth_callback,
            10
        )

        # Publicar distancias promedio por zona (en metros)
        self.pub_left = self.create_publisher(Float32, '/zone_dist_left', 10)
        self.pub_center = self.create_publisher(Float32, '/zone_dist_center', 10)
        self.pub_right = self.create_publisher(Float32, '/zone_dist_right', 10)

        self.get_logger().info('DepthZonesNode listo: /kinect/depth/image_raw -> /zone_dist_*')

    def depth_callback(self, msg: Image):
        # depth en milímetros
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if depth is None:
            return

        depth = depth.astype(np.float32)
        h, w = depth.shape

        # Banda central vertical (evitar techo/piso)
        y0 = int(h * 0.3)
        y1 = int(h * 0.8)
        roi = depth[y0:y1, :]

        # Filtrar valores inválidos
        roi[roi <= 0] = np.nan          # sin dato
        roi[roi > 6000] = np.nan        # >6m ignoramos

        # Dividir campo visual en 3 zonas: izquierda, centro, derecha
        col_third = w // 3
        left_region   = roi[:, :col_third]
        center_region = roi[:, col_third:2*col_third]
        right_region  = roi[:, 2*col_third:]

        def mean_m(region, default=5.0):
            m = np.nanmean(region)
            if np.isnan(m):
                return default  # si no hay dato asumimos lejos
            return m / 1000.0   # mm -> m

        d_left = mean_m(left_region)
        d_center = mean_m(center_region)
        d_right = mean_m(right_region)

        # Publicar distancias promedio en metros
        self.pub_left.publish(Float32(data=float(d_left)))
        self.pub_center.publish(Float32(data=float(d_center)))
        self.pub_right.publish(Float32(data=float(d_right)))

        # ===== Mapa de calor (visual) =====
        depth_clipped = np.copy(roi)
        depth_clipped[np.isnan(depth_clipped)] = 6000.0
        depth_clipped = np.clip(depth_clipped, 0, 3000)      # 0..3m
        norm = (depth_clipped / 3000.0 * 255.0).astype(np.uint8)
        heatmap = cv2.applyColorMap(255 - norm, cv2.COLORMAP_JET)

        cv2.imshow('Depth Heatmap (ROI)', heatmap)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthZonesNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
