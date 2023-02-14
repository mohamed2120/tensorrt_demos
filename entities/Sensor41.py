import time

from lib.entity_types.PureSensor import PureSensor
from lib.entity_types.SensorController import SensorController
from lib.state_machines.FSM import State
from lib.event_bus.decorators import atomic


class Sensor41(SensorController):
    normal_state = State(name='Normal', initial=True)
    sort_out_state = State(name='SortOut')

    rule = [
        (normal_state, sort_out_state),
        (sort_out_state, normal_state),
    ]

    states = [normal_state, sort_out_state]

    def __init__(self, source=None):
        self.source = source
        super(Sensor41, self).__init__(name='sensor_41', states=Sensor41.states, rule=Sensor41.rule)

    def sensing_logic(self):
        while True:
            current_41 = self.source.current
            # print(current_41, "current_41")
            # (1.5 + 2.2) / 2 = 1.85
            if current_41 > 2.2 and self.get_state == Sensor41.sort_out_state:
                
                self.event_bus.publish('cylinder-detected-at-sensor-41')
                self.transition(Sensor41.normal_state)
            else:
                self.previous_detected = False
            time.sleep(0.1)
            # time.sleep(15)
            # if self.get_state == Sensor41.sort_out_state:
            #     self.event_bus.publish("sensor41")

    # @atomic(device='sensor_41')
    def handel_detector(self, event):
        if event.get('event').get('detection') == 'no-cap' or  event.get('event').get('detection') == 'no-detection':
            
            self.transition(Sensor41.sort_out_state)
        else:
            self.transition(Sensor41.normal_state)
