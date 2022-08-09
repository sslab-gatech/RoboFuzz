from enum import Enum, auto

class FeedbackType(Enum):
    """define feedback types (e.g., which direction to favor)"""
    INC = auto() # favor if value increases
    DEC = auto() # favor if value decreases
    ZERO = auto() # favor if closer to zero
    TARGET = auto() # favor if closer to target value


class Feedback:
    """
    Generic class defining a feedback.
    In practice, "feedback" will be a vector of Feedback instances.
    """

    def __init__(self, name, feed_type, default_value=None):
        self.name = name # name of the feedback attribute
        self.feed_type = feed_type
        self.default_value = default_value
        self.value = default_value
        self.prev_interesting_value = None
        self.interesting_value = None # current interesting value
        self.target_value = None # only if type: TARGET

    def set_target(self, target_value):
        assert (self.feed_type == FeedbackType.TARGET)
        self.target_value = target_value

    def update_value(self, value):
        # self.prev_value = self.value
        self.value = value

    def update_interesting(self, init=False):
        if init:
            self.prev_interesting_value = self.value
            self.interesting_value = self.value
        else:
            self.prev_interesting_value = self.interesting_value
            self.interesting_value = self.value

    def reset(self):
        """reset feedback instance when a bug is found"""
        self.value = self.default_value
        self.prev_interesting_value = None
        self.interesting_value = None

    def is_interesting(self):
        # transition from the initial state to any state is interesting
        if self.interesting_value is None:
            self.update_interesting(init=True)
            return False

        # compare current value to the last value that was considered
        # interesting, and update if the current value is more interesting
        if self.feed_type == FeedbackType.INC:
            if self.interesting_value < self.value:
                self.update_interesting()
                return True
            else:
                return False

        elif self.feed_type == FeedbackType.DEC:
            if self.interesting_value > self.value:
                self.update_interesting()
                return True
            else:
                return False

        elif self.feed_type == FeedbackType.ZERO:
            if abs(self.interesting_value - 0) > abs(self.value - 0):
                self.update_interesting()
                return True
            else:
                return False

        elif self.feed_type == FeedbackType.TARGET:
            if abs(self.interesting_value - self.target_value) > abs(self.value - self.interesting_value):
                self.update_interesting()
                return True
            else:
                return False

        return False

    def quantify_interestingness(self):
        # if available, quantify and return the "interestingness"
        pass

