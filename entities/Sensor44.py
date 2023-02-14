import time
import datetime
from lib.entity_types.PureSensor import PureSensor


class Sensor44(PureSensor):

    def __init__(self, source=None):
        self.source = source
        self.last_detected = None
        self.crowded = False
        super(Sensor44, self).__init__(name='sensor_44')

    def sensing_logic(self):
        time.sleep(10)
        print("got here")
        while True:
            current_44 = self.source.current
            # print(current_44)
            if current_44 >= 1.8:
                if self.last_detected is None:
                    self.last_detected = datetime.datetime.now()
                elif (datetime.datetime.now() - self.last_detected).seconds > 1 and not self.crowded:
                    self.crowded = True
                    print("*******croded")
                    self.event_bus.publish("crowded")
            elif current_44 < 1.8:
                if not self.crowded:
                    self.last_detected = None
                else:
                    print("********uncroded")
                    self.crowded = False
                    self.event_bus.publish("uncrowded")
            time.sleep(0.3)


            # (1.5 + 2.2) / 2 = 1.85
            # if current_41 > 1.85 and not self.previous_detected:
            #     self.previous_detected = True
            #     self.event_bus.publish('cylinder-detected-at-sensor-41')
            # else:
            #     self.previous_detected = False
            # time.sleep(0.3)
            # time.sleep(5)
            # self.event_bus.publish("crowded")
            # self.event_bus.publish("uncrowded")
