from styx_msgs.msg import TrafficLight
import tensorflow as tflow
from PIL import Image
import numpy as np
import sys
import cv2

class TLClassifier(object):
    def __init__(self):
        pass
        #TODO load classifier

    def check_specific_color_tl(self, image, color_test1, color_test2):
        # using Hue/Saturation/Value (HSV) color space
        # https://en.wikipedia.org/wiki/HSL_and_HSV
        # works well for base colors (red/blue/green)
        # https://docs.opencv.org/3.1.0/da/d53/tutorial_py_houghcircles.html
        # http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
        

        # constants
        h_delta = 5
        sv_low = 50
        sv_high = 255
        add_weight = 1.0
        extra_weight = 0.0
        gb_kernel = (11,11)
        
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # color 1 range
        lbound = np.array([(color_test1-h_delta), sv_low,sv_low])
        hbound = np.array([(color_test1+h_delta), sv_high,sv_high])
        hsv_mask1 = cv2.inRange(image_hsv, lbound, hbound)

        # color 2 range
        lbound = np.array([(color_test2-h_delta), sv_low,sv_low])
        hbound = np.array([(color_test2+h_delta), sv_high,sv_high])
        hsv_mask2 = cv2.inRange(image_hsv, lbound, hbound)

        # combine
        hsv_combined = cv2.addWeighted(hsv_mask1, add_weight, hsv_mask2, add_weight, extra_weight)

        # blur with large kernel
        hsv_bl = cv2.GaussianBlur(hsv_combined, gb_kernel, 0)

        # hough gradient, find circles separated by enough distance
        find_circles = cv2.HoughCircles(hsv_bl, cv2.HOUGH_GRADIENT,
                                        .5, 40, param1 = 70, param2 = 30,
                                        minRadius = 5, maxRadius = 100)
        return (find_circles is not None)
    
    def run_inference_for_single_image(self, image, graph):
	  with graph.as_default():
	    with tflow.Session() as sess:
	      # Get handles to input and output tensors
	      ops = tflow.get_default_graph().get_operations()
	      all_tensor_names = {output.name for op in ops for output in op.outputs}
	      tensor_dict = {}
	      for key in [
		  'num_detections', 'detection_boxes', 'detection_scores',
		  'detection_classes', 'detection_masks'
	      ]:
		tensor_name = key + ':0'
		if tensor_name in all_tensor_names:
		  tensor_dict[key] = tflow.get_default_graph().get_tensor_by_name(
		      tensor_name)
	      if 'detection_masks' in tensor_dict:
		# The following processing is only for single image
		detection_boxes = tflow.squeeze(tensor_dict['detection_boxes'], [0])
		detection_masks = tflow.squeeze(tensor_dict['detection_masks'], [0])
		# Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
		real_num_detection = tflow.cast(tensor_dict['num_detections'][0], tflow.int32)
		detection_boxes = tflow.slice(detection_boxes, [0, 0], [real_num_detection, -1])
		detection_masks = tflow.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
		detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
		    detection_masks, detection_boxes, image.shape[0], image.shape[1])
		detection_masks_reframed = tflow.cast(
		    tflow.greater(detection_masks_reframed, 0.5), tflow.uint8)
		# Follow the convention by adding back the batch dimension
		tensor_dict['detection_masks'] = tflow.expand_dims(
		    detection_masks_reframed, 0)
	      image_tensor = tflow.get_default_graph().get_tensor_by_name('image_tensor:0')

	      # Run inference
	      output_dict = sess.run(tensor_dict,
		                     feed_dict={image_tensor: np.expand_dims(image, 0)})

	      # all outputs are float32 numpy arrays, so convert types as appropriate
	      output_dict['num_detections'] = int(output_dict['num_detections'][0])
	      
	      output_dict['detection_classes'] = output_dict[
		  'detection_classes'][0].astype(np.uint8)
	      output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
	      
	      output_dict['detection_scores'] = output_dict['detection_scores'][0]
	      #print (output_dict['detection_scores'])
	      if 'detection_masks' in output_dict:
		output_dict['detection_masks'] = output_dict['detection_masks'][0]
		
	  return output_dict

    def load_image_into_numpy_array(self, image):
	  im_width = image.shape[0]
          im_height = image.shape[1]
	  return np.array(image.reshape(im_height, im_width, 3))


    def get_classification(self, image, graph):
        print "in classification module"
	output_dict = self.run_inference_for_single_image(image, graph)
	color_dict = {1: {'name': 'red', 'id': 1}, 2: {'name': 'yellow', 'id': 2}, 3: {'name': 'green', 'id': 3}}
	i=0
	ids = [i for i in range(len(output_dict['detection_scores'])) if output_dict['detection_scores'][i] > 0.5]
	print (ids)
	#print(output_dict['detection_scores'])
	#print(output_dict['detection_classes'])
	classes = [output_dict['detection_classes'][element] for element in ids]
	print (classes)

	#print (color_dict[1]['name'])

	output_colors = [color_dict[element]['name'] for element in classes]
	print (output_colors)
	
	if classes.count(1) > 0:
		if ((classes.count(2) >0) or (classes.count(3) >0)):
                   return TrafficLight.UNKNOWN 
                else:
                   return TrafficLight.RED
        if classes.count(2) > 0:
                if ((classes.count(1) >0) or (classes.count(3) >0)):
                   return TrafficLight.UNKNOWN 
                else:
                   return TrafficLight.YELLOW 
        if classes.count(3) > 0:
                if ((classes.count(1)>0) or (classes.count(2)>0)):
                   return TrafficLight.UNKNOWN  
                else :
                   return TrafficLight.GREEN
        if ((classes.count(1)==0) and (classes.count(2)==0) and (classes.count(3)==0)):
                return TrafficLight.UNKNOWN

    def get_cv_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """    
        #TODO implement light color prediction

        # for red use bright red and magenta red
        # base red is [0,255,255] in HSV, using 5, magenta is 175
        if (self.check_specific_color_tl(image, 5, 175)):
            return TrafficLight.RED
        # green seems to work with base green, [60,255,255] in HSV
        elif (self.check_specific_color_tl(image, 60, 60)):
            return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
   
