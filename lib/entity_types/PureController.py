from threading import Thread
from lib.state_machines.FSM import FSM, State


# stateful FSM with locks
class PureController(FSM):
    def __init__(self, name, states, rule):
        self.name = name
        super(PureController, self).__init__(name=name, states=states, rule=rule)

    def handle_event_on_separate_thread(self, event_bus, handler):
        Thread(target=event_bus.subscribe, args=(handler,), daemon=False).start()

    def handel_events(self, event_handler_pairs):
        for event_handler_pair in event_handler_pairs:
            self.handle_event_on_separate_thread(event_bus=event_handler_pair[0], handler=event_handler_pair[1])
