import rclpy,sys,signal,json,csv,time
import numpy as np
from pathlib import Path
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
rclpy.init();n=rclpy.create_node('perception_validation_recorder');out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
rows=[];counts={};states={};mapstats=[];began=None;last={}
def diagnostic(m):
 for s in m.status:
  values={v.key:v.value for v in s.values};states[s.message]=states.get(s.message,0)+1
  if 'processing_ms' in values:rows.append(dict(state=s.message,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,**values))
def cloud(m,label):
 global began
 counts[label]=counts.get(label,0)+1
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if began is None:began=stamp
 if stamp-last.get(label,-1e20)<1:return
 last[label]=stamp
 fields={f.name:f.offset for f in m.fields};data=np.ndarray((m.height,m.width),dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k] for k in ['x','y','z']],'itemsize':m.point_step}),buffer=m.data,strides=(m.row_step,m.point_step));xyz=np.array([data[k].ravel() for k in ['x','y','z']]).T
 np.savez_compressed(out/f'{label}_{stamp-began:07.2f}.npz',points=xyz,stamp=stamp,frame=m.header.frame_id)
def costmap(m):
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;a=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
 mapstats.append([stamp,int((a==100).sum()),int((a==-1).sum()),int((a==0).sum())])
 if stamp-last.get('costmap',-1e20)>5:
  last['costmap']=stamp;np.savez_compressed(out/f'costmap_{stamp:.2f}.npz',grid=a,resolution=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y],stamp=stamp)
n.create_subscription(DiagnosticArray,'/perception/diagnostics',diagnostic,10)
n.create_subscription(DiagnosticArray,'/nav/diagnostics',diagnostic,10)
for label in ['ground','obstacles','operator_removed','clearing']:n.create_subscription(PointCloud2,f'/perception/{label}',lambda m,label=label:cloud(m,label),qos_profile_sensor_data)
n.create_subscription(OccupancyGrid,'/local_costmap/costmap',costmap,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
try:rclpy.spin(n)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:
 if rows:
  with (out/'metrics.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 with (out/'costmap_metrics.csv').open('w') as f:w=csv.writer(f);w.writerow(['stamp','lethal','unknown','free']);w.writerows(mapstats)
 (out/'status.json').write_text(json.dumps(dict(counts=counts,states=states,diagnostics=len(rows),costmaps=len(mapstats)),indent=2));n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
