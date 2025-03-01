import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import pyproj
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
import random

# Houghton Downtown
# START_LAT = 47.12187
# START_LON = -88.56901


# Lucky clover farm
START_LAT = 45.081679
START_LON = -84.774588



wgs84 = pyproj.CRS("EPSG:4326")
utm = pyproj.CRS("EPSG:32616")
# utm = pyproj.CRS(proj='utm', zone=16, ellps='WGS84')


transformer_utm_to_wgs84 = pyproj.Transformer.from_crs(utm, wgs84, always_xy=True)
transformer_wgs84_to_utm = pyproj.Transformer.from_crs(wgs84, utm, always_xy=True)


EARTH_RADIUS = 6371000  # meters
DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 180 / math.pi

class TfToGpsPublisher(Node):
    def __init__(self):
        super().__init__('tf_to_gps_publisher')
        
        self.declare_parameter('base_tf','map')    # declare a parameter name with a default value. 
        self.declare_parameter('target_tf','base_link')
        self.declare_parameter('gps_tf','gnss_link')
        

        self.base_frame = self.get_parameter('base_tf').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_tf').get_parameter_value().string_value
        self.gps_frame = self.get_parameter('gps_tf').get_parameter_value().string_value

        self.get_logger().info(f"{utm.datum}")
        
        self.start_x, self.start_y = transformer_wgs84_to_utm.transform(START_LON, START_LAT)
###################### change the sub topic as needed ##############################
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)  
        
#######################################################################
        self.gps_publisher = self.create_publisher(
            NavSatFix,
            '/fix', 
            10           
        )
    
    def on_timer(self):
        from_frame_rel = self.base_frame
        to_frame_rel = self.target_frame # 'base_link_gt'

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,                
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        self.get_logger().debug(f"Displacement in map frame: x={t.transform.translation.x}, y={t.transform.translation.y}, z={t.transform.translation.z}")
        
        gps_msg = self.convert_to_gps(t.transform.translation.x+random.uniform(-0.03,0.03), t.transform.translation.y+random.uniform(-0.03,0.03), t.transform.translation.z)
        if gps_msg: 
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.position_covariance = [0.1+random.uniform(-0.03,0.03), 0.0, 0.0,
                                           0.0, 0.1+random.uniform(-0.03,0.03), 0.0,
                                           0.0, 0.0, 0.1+random.uniform(-0.03,0.03)]
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.gps_publisher.publish(gps_msg)


    def convert_to_gps(self, x, y, z):
        utm_x = self.start_x + x
        utm_y = self.start_y + y
        lon, lat = transformer_utm_to_wgs84.transform(utm_x, utm_y)
        alt = z  # Assuming z is relative to the starting altitude

        gps_msg = NavSatFix()
        # gps_msg.header.frame_id = "map"
        gps_msg.header.frame_id = self.gps_frame  # "jackal_gnss_link"
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        gps_msg.status.status = 1
        gps_msg.status.service = 1
        return gps_msg

def main(args=None):
    rclpy.init(args=args)

    node = TfToGpsPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
