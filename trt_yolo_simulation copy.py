"""trt_yolo.py

This script demonstrates how to do real-time object detection with
TensorRT optimized YOLO engine.
"""


import os
import time
import argparse

import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver

from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO
#!/usr/bin/env python
import RPi.GPIO as GPIO # RPi.GPIO can be referred as GPIO from now
import time

#output pin  22 24 26
#input pin 16 
 
ledPin_green = 22
ledPin_yellow = 24
ledPin_red = 26

input_pin = 7
 
def setup():
        GPIO.setmode(GPIO.BOARD)       # GPIO Numbering of Pins
        GPIO.setup(ledPin_green, GPIO.OUT)   # Set ledPin as output        
        GPIO.setup(input_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        #GPIO.setup(ledPin_green, GPIO.OUT)   # Set ledPin as output
        GPIO.output(ledPin_green, GPIO.LOW)  # Set ledPin to LOW to turn Off the LED
	
        GPIO.setup(ledPin_yellow, GPIO.OUT)   # Set ledPin as output
        GPIO.output(ledPin_yellow, GPIO.LOW)  # Set ledPin to LOW to turn Off the LED

        GPIO.setup(ledPin_red, GPIO.OUT)   # Set ledPin as output
        GPIO.output(ledPin_red, GPIO.LOW)  # Set ledPin to LOW to turn Off the LED

def endprogram():
 
        GPIO.output(ledPin_green, GPIO.LOW)
        GPIO.output(ledPin_yellow, GPIO.LOW)
        GPIO.output(ledPin_red, GPIO.LOW)     
        GPIO.cleanup() 

WINDOW_NAME = 'TrtYOLODemo'


def parse_args():
    """Parse input arguments."""
    desc = ('Capture and display live camera video, while doing '
            'real-time object detection with TensorRT optimized '
            'YOLO model on Jetson')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument(
        '-c', '--category_num', type=int, default=80,
        help='number of object categories [80]')
    parser.add_argument(
        '-t', '--conf_thresh', type=float, default=0.3,
        help='set the detection confidence threshold')
    parser.add_argument(
        '-m', '--model', type=str, required=True,
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish|yolov4-p5]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    args = parser.parse_args()
    return args


def loop_and_detect(cam, trt_yolo, conf_th, vis):
    """Continuously capture images from camera and do object detection.

    # Arguments
      cam: the camera instance (video source).
      trt_yolo: the TRT YOLO object detector instance.
      conf_th: confidence/score threshold for object detection.
      vis: for visualization.
    """
    full_scrn = False
    fps = 0.0
    tic = time.time()
    start=time.time()
    frame_number=0
    do_detection=False
    do_skip=False
    totalObjects=0
    totalCaps=0
    totalNoCaps=0
    setup()
    while True:
        
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            break
        img = cam.read()
        original_img=img.copy()
        if img is None:
            break
        (W,H)=img.shape[:2]
        img = show_fps(img, 0,totalObjects,totalCaps,totalNoCaps)
        img=cv2.resize(img,(960,960))
        cv2.imshow(WINDOW_NAME, img)
        while not do_skip:
            key = cv2.waitKey(1)
            if key == ord('D') or key == ord('d'): 
                do_detection=True
                break
            elif key == ord('P') or key == ord('p'): 
                do_skip=True  
                break  
        no_cap_detected=False
        cap_detected=False
        objected_detected=False

        if do_skip:
            do_skip=False
                
        if do_detection:
            img=original_img.copy()
            boxes, confs, clss = trt_yolo.detect(img, conf_th)
            objects_id=[i for i in range(len(boxes))]
            img = vis.draw_bboxes(img, boxes,objects_id,confs, clss)
            end=time.time()
            fps=0
            totalObjects=totalObjects+len(boxes)
            img = show_fps(img, fps,totalObjects,totalCaps,totalNoCaps)
            do_detection=False
            img=cv2.resize(img,(960,960))
            cv2.imshow(WINDOW_NAME, img)
            
            for class_idx in clss:
                objected_detected=True
                if class_idx==0:
                    totalCaps=totalCaps+1
                    cap_detected=True
                if class_idx==1:
                    totalNoCaps=totalNoCaps+1
                    no_cap_detected=True
                
            if not objected_detected:
                GPIO.output(ledPin_yellow, GPIO.HIGH)
            else:
                GPIO.output(ledPin_yellow, GPIO.LOW)

            if  cap_detected:
                GPIO.output(ledPin_green, GPIO.HIGH)
            else:
                GPIO.output(ledPin_green, GPIO.LOW)            


            if  no_cap_detected:
                GPIO.output(ledPin_red, GPIO.HIGH)
            else:
                GPIO.output(ledPin_red, GPIO.LOW)


            while True:
                key = cv2.waitKey(1)
                if key == ord('P') or key == ord('p'): 
                    do_skip=True  
                    break  
            
        




def main():
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('models/%s.trt' % args.model):
        raise SystemExit('ERROR: file (models/%s.trt) not found!' % args.model)

    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    open_window(
        WINDOW_NAME, 'Camera TensorRT YOLO Demo',
        cam.img_width, cam.img_height)
    loop_and_detect(cam, trt_yolo, args.conf_thresh, vis=vis)
    endprogram()
    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
    
