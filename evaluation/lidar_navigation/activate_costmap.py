import rclpy,time
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ManageLifecycleNodes
rclpy.init();n=rclpy.create_node('costmap_activation_gate');first=[]
n.create_subscription(Odometry,'/nav/odom',lambda m:first.append(time.monotonic()) if not first else None,10)
start=time.monotonic()
while not first or time.monotonic()-first[0]<5:
 rclpy.spin_once(n,timeout_sec=.1)
 if time.monotonic()-start>40:raise RuntimeError('No valid nav pose for activation')
c=n.create_client(ManageLifecycleNodes,'/lifecycle_manager_costmap/manage_nodes')
if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Lifecycle manager unavailable')
request=ManageLifecycleNodes.Request();request.command=request.STARTUP;future=c.call_async(request);rclpy.spin_until_future_complete(n,future,timeout_sec=40)
if not future.done() or not future.result().success:raise RuntimeError('Costmap startup failed')
print('Costmap active after cloud clock and navigation pose initialized',flush=True);n.destroy_node();rclpy.shutdown()
