import time
from lib.entity_types.PureController import PureController
from lib.entity_types.SensorController import SensorController
from lib.event_bus.decorators import atomic
from lib.state_machines.FSM import State


class Stopper(SensorController):
    # initial_state = State(name='Initial', initial=True)
    # detecting_state = State(name='Detecting')
    # passing_state = State(name='Passing')

    closed_state = State(name='Closed', initial=True)
    open_state = State(name='Opened')
    # previous_normal = State(name='PreviousNormal', initial=True)
    # previous_sortout = State(name='PreviousSortOut')

    rule = [
        # (initial_state, detecting_state),
        # (initial_state, passing_state),
        # (passing_state, detecting_state),
        # (detecting_state, passing_state),

        # (previous_normal, previous_sortout),
        # (previous_sortout, previous_normal),
        (closed_state, open_state),
        (open_state, closed_state),
    ]

    states = [closed_state, open_state]

    def __init__(self, controller_pin=None):
        self.controller_pin = controller_pin
        self.controller_pin.value = True
        super(Stopper, self).__init__(name='stopper', states=Stopper.states, rule=Stopper.rule)

    def _close(self):
        print("Stopper - closing============")
        self.controller_pin.value = True
        time.sleep(0.2) # resulation of sensing 
        print("Stopper - closed============")
        # print()

    def _open_close(self):
        print("Stopper - open*************")
        self.controller_pin.value = False
        time.sleep(0.6)  # time to pass one cylinder
        self.controller_pin.value = True
        print("Stopper - close**************")
        self.event_bus.publish("open-close-done")
        # print()

    @atomic(device='stopper')
    def sensor_40_handler(self, event):
        # print("got here to open and close", event.get('event'))
        if event.get('event') == 'close-stopper':
            self.transition(Stopper.closed_state, self._close)
        # for test
        # self.transition(Stopper.open_state, self._open_close, None, Stopper.closed_state)


    # @atomic(device='stopper')
    def handle_detection(self, event):
        # print("got signal for opening and closing++++++++++++++++++++++++++++++++++++++")
        self.transition(Stopper.open_state, self._open_close, None, Stopper.closed_state)
       
        

