#!/usr/bin/env python
"""
Custom Mask R-CNN model based on https://pytorch.org/tutorials/intermediate/torchvision_tutorial.html
"""
from mask_rcnn import MaskRCNN
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor

import torch
import torchvision
import numpy as np
import os

if torch.cuda.is_available():
    DEVICE = 'cuda'
else:
    DEVICE = 'cpu'

from PIL import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge

def get_model_instance_segmentation(num_classes, pretrained=False):
    # load an instance segmentation model pre-trained pre-trained on COCO
    model = torchvision.models.detection.maskrcnn_resnet50_fpn(pretrained=pretrained)

    # get number of input features for the classifier
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    # replace the pre-trained head with a new one
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

    # now get the number of input features for the mask classifier
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256
    # and replace the mask predictor with a new one
    model.roi_heads.mask_predictor = MaskRCNNPredictor(in_features_mask,
                                                       hidden_layer,
                                                       num_classes)

    return model

class YcbMaskRCNN(MaskRCNN):
    def __init__(self, model_path, labels):
        self.model_path = None
        self.transform = torchvision.transforms.ToTensor()
        self.labels = labels
        self.load_model(model_path)

    def load_model(self, model_path):
        """ Load model
        loads Mask R-CNN model weights and labels.
        """
        # raise exception if files do not exist
        if not os.path.isfile(model_path):
            raise IOError('Model file not found: {}'.format(model_path))

        # load Mask R-CNN weights
        weights = torch.load(model_path)
        self.net = get_model_instance_segmentation(len(self.labels))
        self.net.load_state_dict(weights)

        self.model_path = model_path
        self.net.to(DEVICE)
        self.net.eval()

    def detect(self, pclmsg, confidence=0.7, mask_confidence=0.8, downsample=2):
        return super(YcbMaskRCNN, self).detect(pclmsg, self.model_path, confidence=0.7, mask_confidence=0.8, downsample=2)


YCB_LABELS_FULL = [
            'marbles', 'clamps', 'plastic lemon',
            'lego duplo', 'cups', 'plate',
            'starkist tuna fish can', 'dice', 'foam brick',
            'plastic peach', 'spam potted meat can', 'windex spray bottle',
            'plastic pear', 'racquetball', 'marbles',
            'baseball', 'scotch brite dobie sponge', 'clamps',
            'cups', 'knife', 'plastic orange',
            'plastic plum', 'cups', 'pitcher base',
            'srub cleanser bottle', 'tennis ball', 'mini soccer ball',
            'adjustable wrench', 'cups', 'toy airplane',
            'toy airplane', 'cups', 'large marker',
            'mug', 'hammer', 'power drill',
            'lego duplo', 'plastic banana', 'cups',
            'lego duplo', 'jell-o chocolate pudding box', 'scissors',
            'wood block', 'domino sugar box', 'toy airplane',
            'lego duplo', 'master chef coffee can', 'golf ball',
            'chain', 'bowl', 'frenchs mustard bottle',
            'plastic strawberry', 'spoon', 'tomato soup can',
            'jell-o strawberry gelatin box', 'lego duplo', 'lego duplo',
            'colored wood blocks', 'cheez-it cracker box', 'soft ball',
            'padlock', 'toy airplane', 'colored wood blocks',
            'lego duplo', 'nine hole peg test', 'spatula',
            'cups', 'skillet lid', 'clamps',
            'cups', 'fork', 'toy airplane',
            'rubiks cube', 'phillips screwdriver', 'cups',
            'flat screwdriver', 'plastic apple', 'skillet',
            'cups'
            ]

if __name__ == '__main__':
    import rospy
    import cv2

    import rospkg
    MODEL_PATH = os.path.join(rospkg.RosPack().get_path('robocup_go_and_get_it'), 'src/utilities/robocup.weights')
    print MODEL_PATH, rospkg.RosPack().get_path('robocup_go_and_get_it')

    # colour stuff
    np.random.seed(69)
    COLOURS = np.random.randint(0, 256, (128,3))
    alpha = 0.5

    rospy.init_node('test')
    mask_rcnn = YcbMaskRCNN(MODEL_PATH, YCB_LABELS_FULL)

    while not rospy.is_shutdown():
        pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        frame, pcl, boxes, clouds, scores, labels, labels_text, masks = mask_rcnn.detect(pclmsg, confidence=0.5)
        
        # output point clouds
        for i, cloud in enumerate(clouds):
            pub = rospy.Publisher('segmentations/{}'.format(i), PointCloud2, queue_size=1)
            pub.publish(cloud)

            #pub = rospy.Publisher('/object_aware_cloud', PointCloud2, queue_size=1)
            #pub.publish(cloud)


        for i, mask in enumerate(masks):
            label = labels[i]
            colour = COLOURS[label]

            # segmentation masks
            binary_mask = mask > 0.5
            frame_coloured = np.array((frame * (1-alpha) + colour * alpha), dtype=np.uint8)
            frame = np.where(binary_mask, frame_coloured, frame)

            # bboxes + info
            x1, y1, x2, y2 = [int(v) for v in boxes[i]]
            cv2.putText(frame, 'confidence: {:.2f}'.format(scores[i]), (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            cv2.putText(frame, 'class: {}'.format(labels_text[i]), (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

        cv2.imshow('test', frame)
        cv2.waitKey(1)
