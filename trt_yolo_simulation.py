"""trt_yolo.py

This script demonstrates how to do real-time object detection with
TensorRT optimized YOLO engine.
"""


import os
import time
import argparse
import sys
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
from threading import Thread

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
        GPIO.setup(ledPin_green, GPIO.OUT)   # Set ledPin as output
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

    parser.add_argument(
        '-save_labels', '--save_labels', type=bool, default=0,
        help='flag for saving images with detections')

    args = parser.parse_args()
    return args

def activate_start_leds(ledPin_yellow,ledPin_red):
    GPIO.output(ledPin_yellow, GPIO.HIGH)
    GPIO.output(ledPin_red, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(ledPin_yellow, GPIO.LOW)
    GPIO.output(ledPin_red, GPIO.LOW)

def loop_and_detect(cam, trt_yolo, conf_th,save_labels,vis):
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
    do_detection=False
    do_skip=False
    totalObjects=0
    totalCaps=0
    totalNoCaps=0
    setup()
    frame_number=0
    is_app_ready=False
    print(("save_labels",save_labels))
    while True:
        
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            break
        img = cam.read()
        if img is None:
            break
        original_img=img.copy()
        if not is_app_ready:
            is_app_ready=True
            start_led_thread = Thread(target=activate_start_leds, kwargs={'ledPin_yellow':ledPin_yellow,'ledPin_red':ledPin_red})
            start_led_thread.daemon = False
            start_led_thread.start()
            

        (W,H)=img.shape[:2]
        img = show_fps(img, 0,totalObjects,totalCaps,totalNoCaps)
        #img=cv2.resize(img,(960,960))
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
        object_detected=False

        if do_skip:
            do_skip=False
        #if user pressed on d to perform detection      
        if do_detection:
            #get copy of original frame
            img=original_img.copy()
            #run the detect function from the trt_yolo instance pass to it the img and the conf_th to filter(remove) boxes with confidence less than conf_th
            #boxes is array representing boxes example [(x_min0,y_min0,x_max0,y_max0),(x_min1,y_min1,x_max1,y_max1),...]
            #confs is array representing the confidences of boxes it's array of floats between 0 and 1 example [0.94,0.45,...]
            #clss is array representing the class of each box  it contains the index not the name of the class example [1,0,...]
            boxes, confs, clss = trt_yolo.detect(img, conf_th)
            #objects_id [0,1,2,...,(len(boxes)-1)]
            objects_id=[i for i in range(len(boxes))]
            #draw boxes on the image
            img = vis.draw_bboxes(img, boxes,objects_id,confs, clss)
            end=time.time()
            fps=0
            #add new objects to totalObjects
            totalObjects=totalObjects+len(boxes)
            #loop through generated classes for each box
            if len(clss) == 0:
                
                object_detected = True
                print("hi_sortout=true")
            else:
                for class_idx in clss:
                #set object_detected to true since there is at least one object(because we entered the loop)
                    object_detected=False
                    print("start_detect",*clss)
                #if it's first class increment the caps count
                    if class_idx==0:
                        totalCaps=totalCaps+1
                        cap_detected=True
                        print("cap",*clss)
                #if it's second class increment the nocaps count
                    if class_idx==1:
                        totalNoCaps=totalNoCaps+1
                        no_cap_detected=True
                        print("no_cap",*clss)
                    
                    #do_detection=False
            #draw different counts on img        
            img = show_fps(img, fps,totalObjects,totalCaps,totalNoCaps)
            #initialize the do_detection

            #do_detection = False
            Normal_Mode = False
            
            #show the image in the window
            cv2.imshow(WINDOW_NAME, img)
            #if sazve lbel flag is true we have to save images
            if save_labels:
                frame_number+=1
                box_with_low_conf=False
                for idx in range(len(boxes)):
                    if confs[idx]<0.90:
                        box_with_low_conf=True
                #if there is at least one object detected in this frame save the image with the label file
                if object_detected and box_with_low_conf:
                    #save the image
                    cv2.imwrite('./auto-labeling/images/frame_'+str(frame_number)+'.jpg',original_img)
                    #create label file
                    label_file='./auto-labeling/labels/frame_'+str(frame_number)+'.txt'
                    #open file in append mode
                    f = open(label_file, "a")
                    #loop through boxes 
                    for idx in range(len(boxes)):
                        #current box
                        bb=boxes[idx]
                        #get height and width of the image 
                        (original_height,original_width)=img.shape[:2]
                        #x represents the vertical axis while y represents the horizontal axis
                        #get top left (x_min,y_min) and bottom right (x_max,y_max)
                        x_min, y_min, x_max, y_max = bb[0], bb[1], bb[2], bb[3]
                        #calculate box height
                        b_height=(x_max-x_min)
                        #calculate box width
                        b_width=(y_max-y_min)
                        #convert absolute values of top left corner of currrent box to relative by dividing by height and width respectively
                        x_min/=original_height
                        y_min/=original_width
                        #convert absolute values of height and width of currrent box to relative by dividing by height and width respectively
                        b_height/=original_height
                        b_width/=original_width
                        #create line consisting of those values
                        line=str(int(clss[idx]))+" "+str(x_min)+" "+str(y_min)+" "+str(b_height)+" "+str(b_width)+"\n"
                        #write the line to file
                        f.write(line)
                    #close the file
                    f.close()
                #else save only the image 
                else:
                    #save the image that the model doesn't detect a single box in it in non_labeled_images folder
                    cv2.imwrite('./auto-labeling/non_labeled_images/frame_'+str(frame_number)+'.jpg',original_img)


            if not object_detected:
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
    try:
        if not cam.isOpened():
            raise SystemExit('ERROR: failed to open camera!')

        cls_dict = get_cls_dict(args.category_num)
        vis = BBoxVisualization(cls_dict)
        trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)



        open_window(
            WINDOW_NAME, 'Camera TensorRT YOLO Demo',
            cam.img_width, cam.img_height)
        save_labels=args.save_labels
        loop_and_detect(cam, trt_yolo, args.conf_thresh,save_labels,vis=vis)
        endprogram()
        cam.release()
        cv2.destroyAllWindows()
    except KeyboardInterrupt:
        endprogram()
        cv2.destroyAllWindows()
        cam.release()
        sys.exit(0)



if __name__ == '__main__':
    main()
    
