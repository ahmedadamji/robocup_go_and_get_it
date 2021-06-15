#!/usr/bin/python
from ycb_mask_rcnn import *

if __name__ == '__main__':
    import rospy
    import ros_numpy
    import cv2

    import rospkg
    MODEL_PATH = os.path.join(rospkg.RosPack().get_path('robocup_go_and_get_it'), 'src/utilities/robocup.weights')
    print MODEL_PATH, rospkg.RosPack().get_path('robocup_go_and_get_it')

    rospy.init_node('test')
    mask_rcnn = YcbMaskRCNN(MODEL_PATH, YCB_LABELS_FULL)

    pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    frame, pcl, boxes, clouds, scores, labels, labels_text, masks = mask_rcnn.detect(pclmsg)

    target = None
    for i, label in enumerate(labels_text):
        if scores[i] > 0.5:
            print label
            if 'mustard' in label:
                target = i

    if target is not None:
        pclmsg = clouds[target]
        pcl = np.fromstring(pclmsg.data, dtype=np.float32)
        pcl = pcl.reshape(pclmsg.height, pclmsg.width, -1)

        cloud = np.zeros(pclmsg.height, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
        cloud['x'] = pcl[:,:,0].flatten()
        cloud['y'] = pcl[:,:,1].flatten()
        cloud['z'] = pcl[:,:,2].flatten()

        # Publish point cloud and nonplanar indices.
        from gpd.msg import CloudIndexed
        from std_msgs.msg import Header, Int64
        from geometry_msgs.msg import Point

        gpd_pub = rospy.Publisher('cloud_indexed', CloudIndexed, queue_size=1)
        pcl_pub = rospy.Publisher('gpd_segmented_cloud', PointCloud2, queue_size=1, latch=1)

        pclmsg_out = ros_numpy.msgify(PointCloud2, cloud)
        pclmsg_out.header = pclmsg.header

        msg = CloudIndexed()
        msg.cloud_sources.cloud = pclmsg_out
        msg.cloud_sources.view_points.append(Point(0,0,0))
        for i in xrange(cloud.shape[0]):
            msg.cloud_sources.camera_source.append(Int64(0))
            msg.indices.append(Int64(i))

        gpd_pub.publish(msg)
        pcl_pub.publish(pclmsg_out)


        # create moveit collision object
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose

        co_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=100)
        rospy.sleep(1)

        co = CollisionObject()
        co.header = pclmsg.header
        co.id = 'target_object'
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = primitive.BOX
        primitive.dimensions = [0.02, 0.02, 0.02]

        no_samples = 200
        indices = np.random.randint(0, cloud.shape[0], no_samples)
        for i in indices:
            x,y,z = cloud[i]
            primitive_pose = Pose()
            primitive_pose.position.x = x
            primitive_pose.position.y = y
            primitive_pose.position.z = z
            primitive_pose.orientation.x = 0
            primitive_pose.orientation.y = 0
            primitive_pose.orientation.z = 0
            primitive_pose.orientation.w = 1
            co.primitives.append(primitive)
            co.primitive_poses.append(primitive_pose)

        co_pub.publish(co)

        print 'Published cloud with', len(msg.indices), 'indices, CTRL+C to exit'
        rospy.spin()
