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
from utils.CentroidTracker import  CentroidTracker

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
    totalObjects=0
    (W,H)=(100,100)
    while True:
        
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            break
        img = cam.read()
        if img is None:
            break

        (W,H)=img.shape[:2]
        break
    print(W,H)
    # initialize our centroid tracker and frame dimensions
    tracker=CentroidTracker(16,5,W/2)
    while True:
        
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            break
        img = cam.read()
        if img is None:
            break

        (W,H)=img.shape[:2]
        if frame_number%2==0:

            #draw counting line on img
            line_thickness = 2
            start_h=0
            start_w=int(W/2)
            end_h=H
            end_w=int(W/2)
            cv2.line(img, (start_h, start_w), (end_h, end_w), (0, 255, 0), thickness=line_thickness)

            boxes, confs, clss = trt_yolo.detect(img, conf_th)
            

            # update our centroid tracker using the computed set of bounding
            # box rectangles
            objects = tracker.update(boxes)
            totalObjects=tracker.getTotalObjects()
            objects_id=[]
            # loop over the tracked objects
            for (objectID, centroid) in objects.items():
                objects_id.append(objectID)            

            img = vis.draw_bboxes(img, boxes,objects_id,confs, clss)
           

            end=time.time()
            fps=(frame_number/(end-start))
            img = show_fps(img, fps,totalObjects)
            img=cv2.resize(img,(960,960))
            cv2.imshow(WINDOW_NAME, img)
            toc = time.time()
            #curr_fps = 1.0 / (toc - tic)
            # calculate an exponentially decaying average of fps number
            #fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
            tic = toc
            key = cv2.waitKey(1)
            if key == 27:  # ESC key: quit program
                break
            elif key == ord('F') or key == ord('f'):  # Toggle fullscreen
                full_scrn = not full_scrn
                set_display(WINDOW_NAME, full_scrn)
        frame_number=frame_number+1


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

    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
