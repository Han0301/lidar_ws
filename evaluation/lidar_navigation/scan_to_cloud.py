"""Evaluation only: ideal 3-D returns from loopback map scans, no hardware model."""
import rclpy,numpy as np
from sensor_msgs.msg import PointCloud2,LaserScan,PointField
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('ideal_scan_cloud');pub=n.create_publisher(PointCloud2,'/simulation/cloud_body',qos_profile_sensor_data)
def convert(scan):
 rows=[]
 for i,r in enumerate(scan.ranges):
  angle=scan.angle_min+i*scan.angle_increment;c=np.cos(angle);s=np.sin(angle)
  limit=min(float(r),scan.range_max) if np.isfinite(r) else scan.range_max
  for distance in np.arange(.6,min(limit-.12,6.0),.20):rows.append([distance*c,distance*s,-.30,20.])
  if np.isfinite(r) and r<scan.range_max:
   for z in np.arange(-.30,.95,.10):rows.append([r*c,r*s,z,30.])
 a=np.array(rows,dtype='<f4').reshape(-1,4);msg=PointCloud2();msg.header=scan.header;msg.height=1;msg.width=len(a);msg.fields=[PointField(name=name,offset=j*4,datatype=PointField.FLOAT32,count=1) for j,name in enumerate(['x','y','z','intensity'])];msg.is_bigendian=False;msg.point_step=16;msg.row_step=16*len(a);msg.is_dense=True;msg.data=a.tobytes();pub.publish(msg)
n.create_subscription(LaserScan,'/scan',convert,qos_profile_sensor_data)
try:rclpy.spin(n)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:n.destroy_node()
