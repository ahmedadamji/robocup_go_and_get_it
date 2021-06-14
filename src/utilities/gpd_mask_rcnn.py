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
        pclmsg = clouds[i]
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

        pub = rospy.Publisher('cloud_indexed', CloudIndexed, queue_size=1)

        msg = CloudIndexed()
        header = Header()
        header.frame_id = "/base_link"
        header.stamp = rospy.Time.now()
        msg.cloud_sources.cloud = ros_numpy.msgify(PointCloud2, cloud)
        msg.cloud_sources.view_points.append(Point(0,0,0))
        for i in xrange(cloud.shape[0]):
            msg.cloud_sources.camera_source.append(Int64(0))
            msg.indices.append(Int64(i))

        pub.publish(msg)
        rospy.sleep(2)
        print 'Published cloud with', len(msg.indices), 'indices'
