import datetime
import time
from threading import Lock, get_ident


class EventBus:
    def __init__(self):
        self.events = []
        self.lock = Lock()
        self.subscribers = []
        self.subscriber = None
        self.running = False

    def publish(self, event):
        events = []
        for subscriber in self.subscribers:
            events.append({
                "thread_id": subscriber.get('thread_id', None),
                "event": event,
                "time": datetime.datetime.now(),
            })
        self.lock.acquire()
        self.events = events
        self.lock.release()

    def subscribe(self, func):
        self.subscribers.append({
            'thread_id': get_ident(),
            'func': func,
        })

        if not self.running:
            self._run()

    def get_and_remove_event(self):
        result = None
        self.lock.acquire()
        for idx, event in enumerate(self.events):
            if event.get('thread_id') == get_ident():
                result = event
                del self.events[idx]
        self.lock.release()
        return result

    def get_registered_func(self):
        func = None
        for idx, subscriber in enumerate(self.subscribers):
            if subscriber.get('thread_id') == get_ident():
                func = subscriber.get('func', None)
        return func

    def _run(self):
        while True:
            if len(self.events) <= 0:
                time.sleep(.02)
                continue

            event = self.get_and_remove_event()
            if event is None:
                time.sleep(.02)
                continue

            func = self.get_registered_func()
            if func is not None:
                func(event)
