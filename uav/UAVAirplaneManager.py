
class UAVAirplaneManager:
    """无人机管理器类
    合并管理 FH0A 和 FH0C 两种无人机
    通过端口号区分不同无人机
    例如：
        FH0C:COM3
        FH0A:COM6
    通过前缀调用对应的 AirplaneManager 实例
    """

    def __init__(self):
        """初始化UAV管理器，导入FH0A和FH0C的AirplaneManager"""
        from .FH0A.AirplaneManagerAdapter import get_airplane_manager as get_fh0a_manager
        from .FH0C.AirplaneManagerAdapter import get_airplane_manager as get_fh0c_manager

        self.fh0a_manager = get_fh0a_manager()
        self.fh0c_manager = get_fh0c_manager()

    def _parse_port(self, id: str):
        """解析端口字符串，返回(类型, 端口)
        例如: "FH0C:COM3" -> ("FH0C", "COM3")
             "FH0A:COM6" -> ("FH0A", "COM6")
        """
        if ':' in id:
            parts = id.split(':', 1)
            drone_type = parts[0].upper()
            port = parts[1]
            return drone_type, port
        else:
            # 如果没有前缀，默认使用FH0A
            return "FH0A", id

    def _get_manager(self, id: str):
        """根据ID获取对应的manager"""
        drone_type, _ = self._parse_port(id)
        if drone_type == "FH0C":
            return self.fh0c_manager
        else:
            return self.fh0a_manager

    def ping(self):
        """检查连接状态"""
        return {'ok': True, 'r': ''}

    def ping_volatile(self):
        """检查临时连接状态"""
        return {'ok': True, 'r': ''}

    def start(self):
        """启动管理器"""
        return {'ok': True, 'r': ''}

    def get_airplane(self, id: str):
        """获取飞机对象
        :param id: 格式为 "类型:端口" 例如 "FH0C:COM3" 或 "FH0A:COM6"
        """
        drone_type, port = self._parse_port(id)
        manager = self._get_manager(id)
        return manager.get_airplane(port)

    def get_airplane_extended(self, id: str):
        """获取扩展飞机对象
        :param id: 格式为 "类型:端口" 例如 "FH0C:COM3" 或 "FH0A:COM6"
        """
        drone_type, port = self._parse_port(id)
        manager = self._get_manager(id)
        return manager.get_airplane_extended(port)

    def sleep(self, time):
        """休眠指定时间"""
        from time import sleep
        sleep(time)

    def flush(self):
        """刷新飞机数据"""
        self.fh0a_manager.flush()
        self.fh0c_manager.flush()
        return None

    def destroy(self):
        """销毁所有无人机连接"""
        self.fh0a_manager.destroy()
        self.fh0c_manager.destroy()


airplane_manager_singleton = UAVAirplaneManager()


def get_airplane_manager():
    """获取AirplaneManager单例"""
    return airplane_manager_singleton

