"""Publish only explicitly reviewed operator boxes at the cloud's own stamp.
JSON: {"boxes": [{"id": "holder", "start_s": 20, "end_s": 230,
"frame": "body", "center": [-.625,.275,.325], "size": [.55,1.05,.85]}]}.
Coordinates and identity are human annotations, not automatic pedestrian detection.
"""
import argparse,json,rclpy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker,MarkerArray
from rclpy.qos import qos_profile_sensor_data
parser=argparse.ArgumentParser();parser.add_argument('labels');args=parser.parse_args();labels=json.load(open(args.labels))['boxes']
for b in labels:
 if len(b['center'])!=3 or len(b['size'])!=3 or min(b['size'])<=0:raise ValueError('Invalid reviewed box')
rclpy.init();n=rclpy.create_node('reviewed_operator_annotations');publisher=n.create_publisher(MarkerArray,'/perception/operator_regions',3);first=None
def annotate(cloud):
 global first
 stamp=cloud.header.stamp.sec+cloud.header.stamp.nanosec*1e-9
 if first is None:first=stamp
 elapsed=stamp-first;output=MarkerArray()
 for i,b in enumerate(labels):
  if not b['start_s']<=elapsed<=b['end_s']:continue
  m=Marker();m.header.stamp=cloud.header.stamp;m.header.frame_id=b['frame'];m.id=i;m.ns=b['id'];m.type=Marker.CUBE;m.action=Marker.ADD;m.pose.orientation.w=1.;m.pose.position.x,m.pose.position.y,m.pose.position.z=[float(x) for x in b['center']];m.scale.x,m.scale.y,m.scale.z=[float(x) for x in b['size']];m.color.r=1.;m.color.a=.3;output.markers.append(m)
 publisher.publish(output)
n.create_subscription(PointCloud2,'/cloud_registered_body',annotate,qos_profile_sensor_data)
try:rclpy.spin(n)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:n.destroy_node()
