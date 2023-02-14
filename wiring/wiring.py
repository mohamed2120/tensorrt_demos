from entities.Sensor40 import Sensor40
from entities.Stopper import Stopper
from entities.Camera import Camera
from entities.Pusher import Pusher
from entities.Sensor41 import Sensor41
from entities.Detector import Detector
from entities.Sensor44 import Sensor44
from entities.ImageSaver import ImageSaver
from utils.visualization import BBoxVisualization
from utils.yolo_classes import get_cls_dict

import pycuda.autoinit

import board
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
import digitalio

from utils.yolo_with_plugins import TrtYOLO


i2c_bus = board.I2C()

stopper_01_pin = board.D19
stopper_01 = digitalio.DigitalInOut(stopper_01_pin)
stopper_01.direction = digitalio.Direction.OUTPUT

pusher_01_pin = board.D26
pusher_01 = digitalio.DigitalInOut(pusher_01_pin)
pusher_01.direction = digitalio.Direction.OUTPUT



def wire_entities(args, cam, window_name):
    #print(args.model, args.category_num, args.letter_box)
    print(args, '*********************************')
    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box, cuda_ctx=pycuda.autoinit.context)

    sensor_40 = Sensor40(source=INA219(i2c_bus, 0x40))

    sensor_44 = Sensor44(source=INA219(i2c_bus, 0x44))

    camera = Camera(args=args, window_name=window_name, cam=cam)


    camera.handel_events(event_handler_pairs=[(sensor_40.get_sensor_event_bus(), camera.handel_sensor_40)])
    detector = Detector(trt_yolo=trt_yolo, conf_th=args.conf_thresh, args=args, vis=vis)
    detector.handel_events(event_handler_pairs=[(camera.get_sensor_event_bus(), detector.handel_camera_event)])

    stopper = Stopper(stopper_01)
    stopper.handel_events(event_handler_pairs=[
        (sensor_40.get_sensor_event_bus(), stopper.sensor_40_handler),
        (detector.get_sensor_event_bus(), stopper.handle_detection),
    ])

    sensor_41 = Sensor41(source=INA219(i2c_bus, 0x41))
    sensor_41.handel_events(event_handler_pairs=[(detector.get_sensor_event_bus(), sensor_41.handel_detector)])

    pusher = Pusher(pusher_01)
    pusher.handel_events(event_handler_pairs=[
        (detector.get_sensor_event_bus(), pusher.handel_detector),
        (sensor_41.get_sensor_event_bus(), pusher.handle_sensor_41_detection)
    ])

    sensor_40.handel_events(event_handler_pairs=[
        (detector.get_sensor_event_bus(), sensor_40.handel_detector),
        (pusher.get_sensor_event_bus(), sensor_40.handel_pusher),
        (sensor_44.get_sensor_event_bus(), sensor_40.handel_sensor_44),
    ])


    if args.save_labels:
        #print(detector.get_sensor_event_bus(), '======================================')
        image_saver = ImageSaver()
        image_saver.handel_events(event_handler_pairs=[
            (detector.get_sensor_event_bus(), image_saver.handel_detector),
        ])