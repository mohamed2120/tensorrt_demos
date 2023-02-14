from threading import Lock


class State:
    def __init__(self, name, initial=False):
        self.name = name
        self.initial = initial

    def __eq__(self, other):
        return self.name == other.name

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return self.name


class FSM:
    def __init__(self, name, states, rule):
        self.states = states
        self.rule = rule
        self.state = None
        self.name = name
        self.state_lock = Lock()
        for state in self.states:
            if state.initial:
                self.state_lock.acquire()
                self.state = state
                self.state_lock.release()
        # todo raise exception if no initial state is provided/ multiple initial states provided

    def _is_allowed(self, starting_state, ending_state):
        for rule_tuple in self.rule:
            if rule_tuple[0] == starting_state and rule_tuple[1] == ending_state:
                return True
        return False

    @property
    def get_state_free(self):
        """
            # NOTE: get_state_free doesn't require lock to get the state
            this function might return out of date State, only intended
            to be used in a function that is passed to self.transition.
        :return: State
        """
        return self.state

    @property
    def get_state(self):
        """
            # NOTE: get_state will require lock to get the state
            using get_state inside a function that is passed to
            self.transaction will CAUSE DEADLOCK!!!
        :return: State
        """
        self.state_lock.acquire()
        state = self.state
        self.state_lock.release()
        return state

    def transition(self, state, func = None, args=None,
                   final_state = None):
        """
            # NOTE: transition will require lock to get the state
            using transition inside a function that is passed to
            self.transaction will CAUSE DEADLOCK!!!
        :return: None
        """
        if args is None:
            args = []
        result = False
        self.state_lock.acquire()
        if self._is_allowed(starting_state=self.state, ending_state=state):
            self.state = state
            result = True
            if func is not None:
                func() if args is None else func(*args)
            if final_state is not None:
                self.state = final_state
        else:
            # todo raise exception if transition is not allowed
            # print(f"Error: {self.name} transition from '{self.state}' is not allowed to '{state}'")
            print()
        self.state_lock.release()
        return result
