from enum import Enum


class QueueSignal(Enum):
    # stop send
    SHUTDOWN = 0
    # send a command
    CMD = 1
    # clean last send command (to stop re-send logic)
    CLEAN = 2
    pass