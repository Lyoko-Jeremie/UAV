from .SerialThread import SerialThread
from typing import List, Dict, Any, Tuple, Union, Literal


class FH0CManager(object):
    map: Dict = {}

    def __init__(self):
        pass

    def new_port(self, port: str):
        self.map[port] = SerialThread(port=port)
        pass

    def get(self, port: str):
        return self.map[port]

    def shutdown(self):
        for p in self.map:
            p.shutdown()
            pass
        pass

    pass
