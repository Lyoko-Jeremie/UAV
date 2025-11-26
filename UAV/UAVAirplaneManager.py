
class UAVAirplaneManager:
    """无人机管理器类
    合并管理 FH0A 和 FH0C 两种无人机
    通过端口号区分不同无人机
    例如：
        FH0C:COM3
        FH0A:COM6
    通过前缀调用对应的 AirplaneManager 实例
    """

    # TODO impl this
    pass


airplane_manager_singleton = UAVAirplaneManager()


def get_airplane_manager():
    """获取AirplaneManager单例"""
    return airplane_manager_singleton

