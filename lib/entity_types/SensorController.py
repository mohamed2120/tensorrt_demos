from lib.state_machines.FSM import State
from lib.event_bus.EventBus import EventBus
from . import PureController
from threading import Thread


class SensorController(PureController.PureController):
    def __init__(self, name, states, rule):
        self.name = name
        super(SensorController, self).__init__(name=name, states=states, rule=rule)
        self.event_bus = EventBus()
        self.do_sensing()

    def sensing_logic(self):
        # implement logic in child class
        pass

    def get_sensor_event_bus(self):
        return self.event_bus

    def do_sensing(self):
        Thread(target=self.sensing_logic, daemon=False).start()
