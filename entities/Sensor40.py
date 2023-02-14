import time

from lib.entity_types.PureSensor import PureSensor
from lib.entity_types.SensorController import SensorController
from lib.state_machines.FSM import State
from lib.event_bus.decorators import atomic
from threading import Lock


class Sensor40(SensorController):
    running = State(name='Running', initial=True)
    waiting_for_detection = State(name='WaitingForDetection')
    sort_out_mode = State(name="SortoutMode")
    waiting_for_sortout = State(name="WaitingForSortout")

    crowded_normal = State(name='CrowdedNormal')
    crowded_sort_out = State(name='CrowdedSortout')
    crowded_while_waiting_for_detection = State(name='CrowdedWhileWaitingForDetection')

    rule = [
        (running, waiting_for_detection),  # when sensed new cylinder
        (waiting_for_detection, running),  # when detector detected normal mode
        (waiting_for_detection, sort_out_mode),  # when detector detected sortout mode
        (sort_out_mode, running),  # when pusher completes
        (sort_out_mode, waiting_for_sortout),  # when sensed new cylinder before sortout done
        (waiting_for_sortout, waiting_for_detection),  # when pusher completes

        # crowded happening
        (running, crowded_normal),
        (waiting_for_detection, crowded_while_waiting_for_detection),
        (sort_out_mode, crowded_sort_out),
        (waiting_for_sortout, crowded_sort_out),

        # waiting camera detection
        (crowded_while_waiting_for_detection, crowded_normal),
        (crowded_while_waiting_for_detection, crowded_sort_out),

        # uncrowded
        (crowded_normal, running),  # start detecting
        (crowded_sort_out, running),  # time delay before starting detecting
        (crowded_sort_out, crowded_normal)
    ]

    states = [running, waiting_for_detection, sort_out_mode, waiting_for_sortout,crowded_normal,crowded_sort_out,crowded_while_waiting_for_detection]

    def __init__(self, source=None):
        self.source = source
        self.previous_gap_detected = True
        super(Sensor40, self).__init__(name='sensor_40', states=Sensor40.states, rule=Sensor40.rule)

    def _new_detected(self, current_40):
        if current_40 > 1.4 and self.previous_gap_detected:
            self.previous_gap_detected = False
            return True
        elif current_40 <= 1.4:
            # time.sleep(0.6) # i add that
            self.previous_gap_detected = True
        return False

    def sensing_logic(self):
        time.sleep(15) # todo, this is for waiting camara and display initialization
        print("started sensing")
        
        while True:
            # if self.get_state == Sensor40.sort_out_mode:
            print("state: ", self.get_state) # important bc we will know what its trying to do
            # print("current: " , self.source.current)
            # print("previous_gap_detected: " , self.previous_gap_detected)
            if self._new_detected(current_40=self.source.current):
                # print("new detecte////////////////////////////")
            # if self._new_detected(current_40=3):
                if self.get_state == Sensor40.running:
                    time.sleep(0.2)
                    self.event_bus.publish('start-detecting')
                    self.transition(Sensor40.waiting_for_detection)
                if self.get_state == Sensor40.sort_out_mode:
                    self.transition(Sensor40.waiting_for_sortout)
            time.sleep(0.1)

            # if current_40 > 1.9 and self.previous_gap_detected:
            #     self.previous_detected = True
            #     self.previous_gap_detected = False
            #     if self.get_state == Sensor40.run_state:
            #         self.event_bus.publish('cylinder-detected-at-sensor-40')
            #         self.transition(Sensor40.stop_state)
            # elif current_40 <= 1.9:
            #     self.previous_gap_detected = True
            #     self.previous_detected = False
            # time.sleep(0.3)

    # @atomic(device='sensor_40')
    def handel_detector(self, event):
        if event.get('event').get('detection') == 'no-cap' or  event.get('event').get('detection') == 'no-detection':
            if self.get_state == Sensor40.waiting_for_detection:
                self.transition(Sensor40.sort_out_mode)
                print("SortOut Mode")
            elif self.get_state == Sensor40.crowded_while_waiting_for_detection:
                self.transition(Sensor40.crowded_sort_out)
        else:
            if self.get_state == Sensor40.waiting_for_detection:
                self.transition(Sensor40.running)
            elif self.get_state == Sensor40.crowded_while_waiting_for_detection:
                self.transition(Sensor40.crowded_normal) # **fix**

    # @atomic(device='sensor_40')
    def handel_pusher(self, event):
        if self.get_state == Sensor40.waiting_for_sortout:
            self.event_bus.publish('start-detecting')
            self.transition(Sensor40.waiting_for_detection)
        elif self.get_state == Sensor40.sort_out_mode:
            self.transition(Sensor40.running)
        elif self.get_state == Sensor40.crowded_sort_out:
            # print("got here -----------------------------------")
            self.transition(Sensor40.crowded_normal)
        # print(self.get_state)
        # print(self.get_state == Sensor40.crowded_sort_out)

    # @atomic(device='sensor_40')
    def handel_sensor_44(self, event):
        if event.get('event') == 'crowded':
            if self.get_state == Sensor40.running:
                self.transition(Sensor40.crowded_normal)
            elif self.get_state == Sensor40.waiting_for_detection:
                self.transition(Sensor40.crowded_while_waiting_for_detection)
            elif self.get_state == Sensor40.sort_out_mode or self.get_state == Sensor40.waiting_for_sortout:
                self.transition(Sensor40.crowded_sort_out)
            self.event_bus.publish('close-stopper')

        elif event.get('event') == 'uncrowded':
            # restart operation
            if self.get_state == Sensor40.crowded_normal:
                self.transition(Sensor40.running)
                self.previous_gap_detected = True
            elif self.get_state == Sensor40.crowded_sort_out:
                time.sleep(0.5)  # time required to sort
                self.transition(Sensor40.running)
                self.previous_gap_detected = True
            elif self.get_state == Sensor40.crowded_while_waiting_for_detection: # **fix**
                self.transition(Sensor40.waiting_for_detection) # **fix**

    # @atomic(device='sensor_40')
    # def handel_stopper(self, event):
    #     self.transition(Sensor40.running)