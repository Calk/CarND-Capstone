import rospy
from styx_msgs.msg import TrafficLight

import numpy as np
import matplotlib.pyplot
import matplotlib.colors

import os
import urllib
import sys
import tarfile
import tensorflow as tf 



def get_tensors(graph):
    with graph.as_default():
        ops = tf.get_default_graph().get_operations()
        all_tensor_names = {output.name for op in ops for output in op.outputs}
        image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
        tensor_names = ['num_detections:0', 'detection_boxes:0', 'detection_scores:0', 'detection_classes:0']
        tensor_dict = {}
        for tensor_name in tensor_names:
            tensor_dict[tensor_name] = tf.get_default_graph().get_tensor_by_name(
                          tensor_name)
    return image_tensor, tensor_dict

class TLClassifier(object):
    def __init__(self):
        GRAPH_NAME = 'light_classification/ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb'
        
        self.detection_graph = tf.Graph()
        
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(GRAPH_NAME, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            
            self.image_tensor, self.tensors = get_tensors(self.detection_graph)
            self.session = tf.Session()
        
    def detect_color(self, img):
        hsv_img = matplotlib.colors.rgb_to_hsv(img)
        v = hsv_img[:,:,2].mean(axis=1)
        if v[0:int(v.shape[0]/2)].mean() > v[int(v.shape[0]/2):].mean():
            return TrafficLight.RED
        else:
            return TrafficLight.GREEN
    
    def classify_image(self, image, output_dict):
        boxes = [box for box, cls, score in zip(output_dict['detection_boxes:0'][0],
           output_dict['detection_classes:0'][0],
           output_dict['detection_scores:0'][0]) if score > 0.5 and cls == 10]
           
        rospy.loginfo("Found {} possible lights". format(len(boxes)))
        for box in boxes:
            x_0, y_0, x_1, y_1 = box
            subimg = image[int(image.shape[0]*x_0):int(image.shape[0]*x_1),int(image.shape[1]*y_0):int(image.shape[1]*y_1),:]
            return self.detect_color(subimg)
        
        return TrafficLight.UNKNOWN

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        output_dict = self.session.run(self.tensors, feed_dict={self.image_tensor: np.expand_dims(image, 0)})
        return self.classify_image(image, output_dict)
