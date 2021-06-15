import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2

data = np.zeros(100, dtype=[
  ('x', np.float32),
  ('y', np.float32),
  ('z', np.float32)
])

msg = ros_numpy.msgify(PointCloud2, data)
msg.header.frame_id = 'map'

rospy.init_node('empty_cloud_pub')
cloud_pub = rospy.Publisher('/object_aware_cloud', PointCloud2, queue_size=1)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    cloud_pub.publish(msg)
    rate.sleep()