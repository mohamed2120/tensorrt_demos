from threading import Thread
from lib.event_bus.EventBus import EventBus


class PureSensor:
    def __init__(self, name):
        self.name = name
        self.event_bus = EventBus()
        self.do_sensing()

    def sensing_logic(self):
        # implement logic in child class
        pass

    def do_sensing(self):
        Thread(target=self.sensing_logic, daemon=False).start()

    def get_sensor_event_bus(self):
        return self.event_bus
