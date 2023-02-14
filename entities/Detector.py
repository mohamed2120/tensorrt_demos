import time

from lib.entity_types.SensorController import SensorController
from lib.state_machines.FSM import State
from lib.event_bus.decorators import atomic


class Detector(SensorController):
    not_working_state = State(name='NotWorking', initial=True)
    waiting_for_img = State(name='ImgWaiting')
    waiting_for_sensor_40 = State(name='WaitingForSensor40')
    detecting = State(name='Detecting')

    rule = [
        # (not_working_state, waiting_for_img),
        # (not_working_state, waiting_for_sensor_40),
        # (waiting_for_img, detecting),
        # (waiting_for_sensor_40, detecting),
        # (detecting, not_working_state),
        (not_working_state, detecting),
        (detecting, not_working_state),
    ]

    states = [not_working_state, waiting_for_img,
              waiting_for_sensor_40, detecting]

    def __init__(self, trt_yolo=None, conf_th=None, vis=None, args=None):
        self.img = None
        self.trt_yolo = trt_yolo
        self.conf_th = conf_th
        self.args = args
        self.vis = vis
        self.totalCaps = 0
        self.totalNoCaps = 0
        self.totalObjects = 0
        super(Detector, self).__init__(name='yolo-detector',
                                       states=Detector.states, rule=Detector.rule)

    def _do_fake_detection(self, img):
        print("detection started")
        time.sleep(3)
        # print(img)
        print("detection done")
        print("waiting time")
        # sort-out time delay to make sure the line is empty between current 40 and 41
        time.sleep(0.5)
        self.event_bus.publish(
            {'detection': 'no-cap', 'meta-data': {'boxeds': 'boxes', 'confs': 'confs', 'clss': 'clss'}})
        # detect_done = False

    def _do_detection(self, original_img):
        try:
            img = original_img.copy()
        except AttributeError as e:
            print('Eerror detector got None image rom camera.')
            return
        # print(img)
        # print(self.conf_th, "conf_th")

        boxes, confs, clss = self.trt_yolo.detect(img, self.conf_th)
        # objects_id = [i for i in range(len(boxes))]
        # img = self.vis.draw_bboxes(img, boxes, objects_id, confs, clss)
        # # end = time.time()
        # fps = 0
        # self.totalObjects = self.totalObjects + len(boxes)
        # img = show_fps(img, fps, self.totalObjects, self.totalCaps, self.totalNoCaps)
        # do_detection = False
        # img = cv2.resize(img, (960, 960))
        # cv2.imshow(WINDOW_NAME, img)
        if len(clss) == 0:
            time.sleep(0.2)  # time
            self.event_bus.publish({'detection': 'no-detection', 'meta-data': {
                                   'img': img, 'boxes': boxes, 'confs': confs, 'clss': clss}})
            print("no-detection")
        elif len(clss) >= 2:
            time.sleep(0.2)  # time
            self.event_bus.publish({'detection': 'no-detection', 'meta-data': {
                                   'img': img, 'boxes': boxes, 'confs': confs, 'clss': clss}})
            print("more 2 class happened")
        else:
            for class_idx in clss:
                # objected_detected = True
                if class_idx == 0:
                    self.totalCaps += 1
                    self.event_bus.publish(
                        {'detection': 'cap', 'meta-data': {'img': img, 'boxes': boxes, 'confs': confs, 'clss': clss}})
                    print("cap", confs)
                    break
                if class_idx == 1:
                    self.totalNoCaps += 1
                    time.sleep(0.2)  # time
                    self.event_bus.publish(
                        {'detection': 'no-cap', 'meta-data': {'img': img, 'boxes': boxes, 'confs': confs, 'clss': clss}})
                    print("no-cap" , confs)
                    break

    @atomic(device='yolo-detector')
    def handel_camera_event(self, event):
        img = event.get('event').get('img')
        self.transition(Detector.detecting, self._do_detection,
                        (img,), Detector.not_working_state)
