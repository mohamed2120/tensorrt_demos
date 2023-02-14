"vesrion 01 sorting Caps app"

"""trt_yolo.py

This script demonstrates how to do real-time object detection with
TensorRT optimized YOLO engine.
"""
"caps"

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
from threading import Thread, Lock


import board
import digitalio
import Jetson.GPIO as GPIO

from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
from ssl import CHANNEL_BINDING_TYPES


# Pin Definition 31-X , 33 , 35 , 37
# pin CEM 6 , 13 , 19 , 26

# Pin setup
Not_detect=False
stopper_01_pin = board.D19
pucher_01_pin = board.D26
stopper_01 = digitalio.DigitalInOut(stopper_01_pin)
stopper_01.direction = digitalio.Direction.OUTPUT
pucher_01 = digitalio.DigitalInOut(pucher_01_pin)
pucher_01.direction = digitalio.Direction.OUTPUT

i2c_bus = board.I2C()

stopper_Sensor = INA219(i2c_bus,0x40)
pusher_Sensor = INA219(i2c_bus,0x41)
crowded_Sensor = INA219(i2c_bus,0x44)


stopper_01_lock = Lock()
pucher_01_lock = Lock()


current_40 = 0
current_41 = 0

current_lock = Lock()
stopper_count = 0
count_crowded = 0
Sortout_Mode = False
Normal_Mode = False
itis_crawded = False

def update_current_40():
    current_lock.acquire()
    global current_40 
    current_40 = stopper_Sensor.current  # current in mA
    # print(current_40)
    # print("--------------------------------------")
    global current_41 
    current_41 = pusher_Sensor.current  # current in mA
    # print(current_41)
    global current_44
    current_44 = pusher_Sensor.current  # current in mA
    current_lock.release()


def do_senceing():
    while True:
        update_current_40()
        # print('**************** done sensing on separated thred*',current_40,current_41)
        time.sleep(0.3)


def Normal_Mode_open_close_stopper(duration):
    stopper_01_lock.acquire()
    stopper_01.value = True
    time.sleep(duration)
    stopper_01.value = False
    # print('stopper_____________________________')
    stopper_01_lock.release()

def open_stopper():
    stopper_01_lock.acquire()
    stopper_01.value = False
    #time.sleep(duration)
    #stopper_01.value = False
    # print('stopper opening_____________________________')
    stopper_01_lock.release()

def close_stopper():
    stopper_01_lock.acquire()
    stopper_01.value = True
    # print("cLOSING STOPPER ========================================")
    stopper_01_lock.release()

def open_close_pusher(duration):
    pucher_01_lock.acquire()
    pucher_01.value = True
    # print('+==============================================')
    time.sleep(0.5)
    pucher_01.value = False
    # print('------------------------------------------------s')
    pucher_01_lock.release()
    


def endprogram():
    stopper_01.value = False
    pucher_01.value = False
    GPIO.cleanup()

WINDOW_NAME = 'Cap_App'

def parse_args():
    """Parse input arguments"""
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

def crowded():
    current_lock.acquire()
    global current_44
    global count_crowded
    global itis_crawded
    count_crowded = 0
    itis_crawded = False

    current_44 = crowded_Sensor.current  # current in mA
    # print("Current_44  : {:7.4f}  mA".format(current_44))
    while current_44 > 1.6 :
        #time.sleep(0.2)
        count_crowded = count_crowded + 1
        # print("count_crowded--------->",count_crowded)
        if count_crowded > 20:    
            itis_crawded = True
            print("crowded")
            
        elif current_44 < 1.6:
            
            itis_crawded = False
            count_crowded = 0
            print("not_crowded")
            
        current_44 = crowded_Sensor.current  # current in mA    
        # print(current_44)
    current_lock.release()

##--------------########################################------------------------############################-------------------------################

def normal_run():
    global stopper_count
    
    global stopper_01

    stopper_01.value = False
    time.sleep(0.5)
    stopper_01.value = True
    print("normal_mode")
    
def sort_out():
    global stopper_count
    global stopper_01
    # global do_detection
    Sortout_Mode = True
    stopper_01.value=False
    time.sleep(0.6)
    stopper_01.value=True
    print("sort out mode")
    while True:     
        current_41=pusher_Sensor.current
        if current_41 > 2:
            pucher_01.value = True
            time.sleep(1.4)
            pucher_01.value = False # red led off
            print("pusher done")
            break
        else:
            time.sleep(0.5)

            
                    
            


def loop_and_detect(cam, trt_yolo, conf_th,save_labels,vis):
    """Continuously capture images from camera and do object detection.
global stopper_count z
    # Arguments
        cam: the camera instance (video source).
        trt_yolo: the TRT YOLO object detector instance.
        conf_th: confidence/score threshold for object detection.
        vis: for visualization.
        save_labels: for saving labels
    """
    
    global stopper_count
    global stopper_01
    global count_crowded
    global itis_crawded
    
    #itis_crawded =False
    fps = 0.0

    #do_detection flag for detection when pressing on d
    
    global Normal_Mode
    
    global Sortout_Mode

    close_gate=False
    do_detection=False

    Once_Detect = False
    #do_skip to skip frame without processing press on p
    do_skip=False
    #for count 
    totalObjects=0
    totalCaps=0
    totalNoCaps=0
    
    frame_number=0
    is_app_ready=False


    while True:
        crowded()
        
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            break
        #grab image from input source
        img = cam.read()
        
        if img is None:
            break

        #get original copy of the img before starting the process
        original_img=img.copy()
        #if the app isn't ready we need to power yellow and red leds and set is_app_ready true so it's done only once

            
        (W,H)=img.shape[:2]
        #first step show the new image with previous state (totalObjects,totalCaps,totalNoCaps)
        img = show_fps(img, 0,totalObjects,totalCaps,totalNoCaps)
        cv2.imshow(WINDOW_NAME, img)
        cv2.waitKey(1)

        #while user didn't press d or p stay in this loop
        while not do_skip:
            
###############################################################################################################################################
            current_40 = stopper_Sensor.current  # current in mA
            # current_41 = pusher_Sensor.current  # current in mA
            if (current_40 > 0.6):
                close_gate=True
                do_detection=True
                break
            if current_40 < 0.6 :
                do_skip=True               
                break  
        #initialize the flags
#########################################################################################################################################################

        no_cap_detected=False
        cap_detected=False
        object_detected=False
        # Normal_Mode = False
        Sortout_Mode = False
        
        # if Normal_Mode:
            # if current_40 > 0.5:
            #     # stopper_01.value = True
            #     # time.sleep(2)
            #     # stopper_01.value = False
            #     Thread(target=Normal_Mode_open_close_stopper, args=(2,), daemon=False).start()


                    
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
            #Sprint("start_detect",*clss,"++++",*boxes,"____",*objects_id )
            #loop through generated classes for each box
            
            for class_idx in clss:
            #set object_detected to true since there is at least one object(because we entered the loop)
                object_detected=True
                #print("start_detect",*clss)
            #if it's first class increment the caps count
                if class_idx==0:
                    totalCaps=totalCaps+1
                    cap_detected=True
                    #print("cap",*clss)
            #if it's second class increment the nocaps count
                if class_idx==1:
                    totalNoCaps=totalNoCaps+1
                    no_cap_detected=True
                    #print("no_cap",*clss)
                do_detection = False

                #do_detection=False
            #draw different counts on img        
            img = show_fps(img, fps,totalObjects,totalCaps,totalNoCaps)
            #initialize the do_detection
            # print("Current_40_3  : {:7.4f}  A".format(current_40))
            # print("Current_41_3  : {:7.4f}  A".format(current_41))
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
                        
            if  not object_detected:
                sort_out()
                
                print("No_detected")
                #print("----",stopper_count)
                
            elif  cap_detected:
                Normal_Mode = True
                normal_run()
                print("cap_detected")
           
            elif  no_cap_detected:
                
                sort_out()
                print("no_cap_detecte")
                


                # while True:
                    
                #     current_40 = stopper_Sensor.current  # current in mA
                #     current_41 = pusher_Sensor.current  # current in mA
                #     print("Current_40_5  : {:7.4f}  A".format(current_40))
                #     print("Current_41_5  : {:7.4f}  A".format(current_41))
                #     cv2.waitKey(1)
                    
                #     if current_40 < 0.4:
                #     #simulations the line
                #         time.sleep(2)
                #         do_skip=True
                #         print("Current_40_2  : {:7.4f}  A".format(current_40))
                #         print("Current_41_2  : {:7.4f}  A".format(current_41))
                #         print("not_detect  : {:7.4f}  A".format(Not_detect))   
                        # break
def main():
    #read input arguments
    args = parse_args()
    time.sleep(0.1)
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('models/%s.trt' % args.model):
        raise SystemExit('ERROR: file (models/%s.trt) not found!' % args.model)


    #create cam object with args input
    cam = Camera(args)
    try:
        if not cam.isOpened():
            raise SystemExit('ERROR: failed to open camera!')
        #cls_dict is classes dictionary
        cls_dict = get_cls_dict(args.category_num)
        #instance of class BBoxVisualization for drawing boxes
        vis = BBoxVisualization(cls_dict)
        #TrtYolo class for processing frames
        trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)


        #windows for displaying processed frame
        open_window(
            WINDOW_NAME, 'Camera TensorRT YOLO Cap',
            cam.img_width, cam.img_height)

        #save_labels flag to save detections in processed images
        save_labels=args.save_labels
        Thread(target=do_senceing, daemon=True).start()
        #function for grabbing and processing frames
        loop_and_detect(cam, trt_yolo, args.conf_thresh,save_labels,vis=vis)
        #end gpio program
        endprogram()
        #release cam and destroy windows
        cam.release()
        cv2.destroyAllWindows()
    except KeyboardInterrupt:
        endprogram()
        #release cam and destroy windows
        cv2.destroyAllWindows()
        cam.release()
        sys.exit(0)



if __name__ == '__main__':
    main()
