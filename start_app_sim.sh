#!/bin/bash
#without saving detections
#sudo python3 trt_yolo_simulation.py --video src/file190914.avi -m yolov4-tiny-416-best  -c 2
#to save detections (images+labels) we use --save_labels 1
#sudo python3 cap_app_copy2.py --video src/cap.avi -m yolov4-tiny-416-best  -c 2 --save_labels 1
#sudo python3 cap_app_copy2.py --onboard 0 -m yolov4-tiny-416-best  -c 2
#trt_yolo_simulation.py
#python3 main.py --video src/cap.avi -m yolov4-tiny-416-best  -c 2 --save_labels 2
#sudo python3 main.py --onboard 0 -m yolov4-tiny-416-best  -c 2
sudo python3 main.py --onboard 0 -m yolov4-tiny-416-best  -c 2 -l -save_labels 0 -d 0


#sudo python3 trt_yolo_simulation.py --video src/file190914.avi -m yolov4-tiny-416-best  -c 2 --save_labels 1 