import time

from lib.entity_types.SensorController import SensorController
from lib.state_machines.FSM import State
from lib.event_bus.decorators import atomic
from utils.display import open_window
import cv2

from threading import Thread


class Camera(SensorController):

    rule = []

    states = []

    def __init__(self, cam, args, window_name):
        self.window_name = window_name
        self.img = None
        self.cam = cam
        self.display_camera = args.display_camera
        super(Camera, self).__init__(name='stopper',
                                     states=Camera.states, rule=Camera.rule)
        Thread(target=self._set_up_camera, args=(args,), daemon=False).start()

    def _set_up_camera(self, args):
        if not self.cam.isOpened():
            raise SystemExit('ERROR: failed to open camera!')

        if self.display_camera:
            open_window(
                self.window_name, 'Camera TensorRT YOLO Demo',
                self.cam.img_width, self.cam.img_height)
            print("camera-done initializing")
        while True:
            img = self.cam.read()
            self.img = img
            if img is None:
                break
            if self.display_camera:
                cv2.imshow(self.window_name, img)
                key = cv2.waitKey(1)

        # # todo do clean up

    @atomic(device='camera')
    def handel_sensor_40(self, event):
        # print(self.img, "got image for detection")
        time.sleep(0.5)  # time before taking the image how much
        if event.get('event') == 'start-detecting':
            while True:
                if not self.img is None:
                    # print(self.img)
                    self.event_bus.publish({"img": self.img})
                    break
                else:
                    time.sleep(0.1)
                    print('retrynig')

    def handel_detector(self, event):
        print(event)
