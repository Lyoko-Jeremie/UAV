"""多机异步飞行示例。

该脚本演示了一个简化的“飞行 -> 图像采集 -> 路径重规划”闭环：
1. 每架无人机异步执行飞行、拍照和规划任务。
2. ConstraintCoordinator 负责统一约束边界和机间最小安全距离。
3. 图像处理逻辑目前是占位实现，便于后续替换为真实 OpenCV 算法。
"""

import asyncio
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from uav import UAVAirplaneManager, get_airplane_manager


@dataclass
class Waypoint:
    """单个航点，包含平面坐标和高度。"""

    x: int
    y: int
    h: int


@dataclass
class BoundaryConstraint:
    """飞行空间边界约束。"""

    x_min: int = 0
    x_max: int = 600
    y_min: int = 0
    y_max: int = 900
    h_min: int = 0
    h_max: int = 300


@dataclass
class FleetConstraintOptions:
    """多机协同约束配置。"""

    boundary: BoundaryConstraint = field(default_factory=BoundaryConstraint)
    min_xy_distance: Optional[float] = 50.0


class ConstraintCoordinator:
    """集中维护多架无人机的占位点，并为新航点做安全修正。"""

    def __init__(self, options: FleetConstraintOptions):
        self.options = options
        self._positions: Dict[str, Waypoint] = {}
        # 多个协程会并发申请航点，使用锁确保位置表更新一致。
        self._lock = asyncio.Lock()

    def _clamp_to_boundary(self, wp: Waypoint) -> Waypoint:
        """将航点限制在允许飞行区域内。"""
        b = self.options.boundary
        return Waypoint(
            x=max(b.x_min, min(b.x_max, int(wp.x))),
            y=max(b.y_min, min(b.y_max, int(wp.y))),
            h=max(b.h_min, min(b.h_max, int(wp.h))),
        )

    def _is_xy_safe(self, drone_id: str, wp: Waypoint) -> bool:
        """检查目标点与其他无人机的水平间距是否满足约束。"""
        threshold = self.options.min_xy_distance
        if threshold is None:
            return True

        for other_id, other_wp in self._positions.items():
            if other_id == drone_id:
                continue
            if math.hypot(wp.x - other_wp.x, wp.y - other_wp.y) < threshold:
                return False
        return True

    def _search_safe_waypoint(self, drone_id: str, target: Waypoint, fallback: Waypoint) -> Waypoint:
        """优先使用目标点，否则在其周围搜索一个满足安全距离的替代点。"""
        threshold = self.options.min_xy_distance
        if threshold is None:
            return target

        if self._is_xy_safe(drone_id, target):
            return target

        fallback = self._clamp_to_boundary(fallback)
        if self._is_xy_safe(drone_id, fallback):
            return fallback

        # 以目标点为中心，按环形离散采样寻找满足最小间距的可行点。
        for radius in (threshold, threshold * 1.5, threshold * 2.0, threshold * 2.5, threshold * 3.0):
            for angle_deg in range(0, 360, 30):
                angle = math.radians(angle_deg)
                candidate = Waypoint(
                    x=int(target.x + radius * math.cos(angle)),
                    y=int(target.y + radius * math.sin(angle)),
                    h=target.h,
                )
                candidate = self._clamp_to_boundary(candidate)
                if self._is_xy_safe(drone_id, candidate):
                    return candidate

        return fallback

    async def reserve_safe_waypoint(self, drone_id: str, target: Waypoint, fallback: Waypoint) -> Waypoint:
        """为无人机登记一个经过约束修正后的航点。"""
        async with self._lock:
            clamped_target = self._clamp_to_boundary(target)
            safe = self._search_safe_waypoint(drone_id, clamped_target, fallback)
            self._positions[drone_id] = safe
            return safe

    async def register_initial_position(self, drone_id: str, initial_wp: Waypoint) -> Waypoint:
        """将初始位置也纳入统一的安全约束管理。"""
        return await self.reserve_safe_waypoint(drone_id, initial_wp, initial_wp)


def process_image_and_plan_next_routes(image_bytes: bytes, last_waypoint: Waypoint) -> List[Waypoint]:
    """OpenCV处理占位函数: 根据回传图像生成下一段航点。"""
    if not image_bytes:
        return [Waypoint(last_waypoint.x, last_waypoint.y + 80, last_waypoint.h)]

    frame = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return [Waypoint(last_waypoint.x, last_waypoint.y + 80, last_waypoint.h)]

    # 占位策略: 用图像亮度决定左右偏移，随后继续前进。
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    shift = 80 if float(gray.mean()) > 127 else -80
    return [
        Waypoint(last_waypoint.x + shift, last_waypoint.y, last_waypoint.h),
        Waypoint(last_waypoint.x + shift, last_waypoint.y + 80, last_waypoint.h),
    ]


class AsyncDroneFeedbackLoop:
    """单架无人机的异步闭环控制器。"""

    def __init__(
        self,
        manager: UAVAirplaneManager,
        airplane_id: str,
        loop: asyncio.AbstractEventLoop,
        stop_event: asyncio.Event,
        coordinator: ConstraintCoordinator,
        seed_waypoints: Optional[List[Waypoint]] = None,
        takeoff_h: int = 120,
        capture_interval_s: float = 1.0,
    ):
        self.manager = manager
        self.airplane_id = airplane_id
        self.loop = loop
        self.stop_event = stop_event
        self.coordinator = coordinator
        self.takeoff_h = takeoff_h
        self.capture_interval_s = capture_interval_s
        self.seed_waypoints = seed_waypoints or []

        self.airplane: Any = manager.get_airplane(airplane_id)
        # 图像队列只保留很少的数据，避免处理速度落后时积压旧画面。
        self.image_queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=2)
        # 路径队列允许短暂缓存未来航点，平衡规划和飞行速度。
        self.route_queue: asyncio.Queue[Waypoint] = asyncio.Queue(maxsize=16)
        self.last_waypoint = Waypoint(0, 0, takeoff_h)
        self.has_taken_off = False

    def _on_image_received(self, image_bytes: bytes) -> None:
        """接收底层回调中的图像数据，并安全转交给事件循环线程。"""
        def _push() -> None:
            if self.image_queue.full():
                try:
                    self.image_queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            self.image_queue.put_nowait(image_bytes)

        # 回调可能不在事件循环线程，使用线程安全投递。
        self.loop.call_soon_threadsafe(_push)

    async def _flight_loop(self) -> None:
        """持续消费规划结果并驱动无人机飞向下一个安全航点。"""
        while not self.stop_event.is_set():
            try:
                wp = await asyncio.wait_for(self.route_queue.get(), timeout=0.5)
            except asyncio.TimeoutError:
                continue

            safe_wp = await self.coordinator.reserve_safe_waypoint(
                self.airplane_id,
                wp,
                self.last_waypoint,
            )
            self.airplane.goto(int(safe_wp.x), int(safe_wp.y), int(safe_wp.h))
            self.last_waypoint = safe_wp
            await asyncio.sleep(0.35)

    async def _capture_loop(self) -> None:
        """按固定周期触发拍照。"""
        while not self.stop_event.is_set():
            self.airplane.cap_image(
                user_receive_callback=self._on_image_received,
                user_progress_callback=lambda _progress, _total: None,
            )
            await asyncio.sleep(self.capture_interval_s)

    async def _plan_loop(self) -> None:
        """读取最新图像并在后台线程中生成新的航点序列。"""
        while not self.stop_event.is_set():
            try:
                image_bytes = await asyncio.wait_for(self.image_queue.get(), timeout=0.5)
            except asyncio.TimeoutError:
                continue

            next_points = await asyncio.to_thread(
                process_image_and_plan_next_routes,
                image_bytes,
                self.last_waypoint,
            )

            for wp in next_points:
                # 若规划速度快于飞行速度，丢弃最旧航点，优先执行最新决策。
                if self.route_queue.full():
                    try:
                        self.route_queue.get_nowait()
                    except asyncio.QueueEmpty:
                        pass
                self.route_queue.put_nowait(wp)

    async def run(self) -> None:
        """执行起飞、任务循环和最终降落的完整生命周期。"""
        tasks: List[asyncio.Task[Any]] = []
        try:
            self.last_waypoint = await self.coordinator.register_initial_position(
                self.airplane_id,
                self.last_waypoint,
            )
            self.airplane.takeoff(self.takeoff_h)
            self.has_taken_off = True
            await asyncio.sleep(4)

            # 起飞后先投递首批航点，尽快进入飞行-感知-重规划循环。
            if self.seed_waypoints:
                for wp in self.seed_waypoints:
                    await self.route_queue.put(wp)
            else:
                await self.route_queue.put(Waypoint(100, 100, self.takeoff_h))
                await self.route_queue.put(Waypoint(100, 200, self.takeoff_h))

            tasks = [
                asyncio.create_task(self._flight_loop()),
                asyncio.create_task(self._capture_loop()),
                asyncio.create_task(self._plan_loop()),
            ]
            await self.stop_event.wait()
        finally:
            for t in tasks:
                t.cancel()
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)

            if self.has_taken_off:
                self.airplane.land()
                await asyncio.sleep(4)
                print(f"[{self.airplane_id}] Landed")


async def manager_flush_pump(manager: UAVAirplaneManager, stop_event: asyncio.Event) -> None:
    """持续刷新底层管理器，驱动通信和状态同步。"""
    while not stop_event.is_set():
        manager.flush()
        await asyncio.sleep(0.2)


async def main_async() -> None:
    """启动多架无人机并让它们在共享约束下并发执行任务。"""
    manager = get_airplane_manager()
    manager.start()
    manager.flush()

    airplane_ids = [
        "FH0C:COM3",
        "FH0C:COM4",
        "FH0C:COM5",
    ]

    # 可配置约束：边界默认 x=[0,600], y=[0,900], h=[0,300]。
    # 将 min_xy_distance 设为 None 可关闭无人机间最小间距约束。
    constraints = FleetConstraintOptions(
        boundary=BoundaryConstraint(),
        min_xy_distance=50.0,
    )
    coordinator = ConstraintCoordinator(constraints)

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()
    workers: List[AsyncDroneFeedbackLoop] = []
    for idx, aid in enumerate(airplane_ids):
        # 为每架无人机分配不同的初始航线，降低起飞后立即冲突的概率。
        lane_x = 100 + idx * 120
        seeds = [
            Waypoint(lane_x, 120, 120),
            Waypoint(lane_x, 240, 120),
        ]
        workers.append(
            AsyncDroneFeedbackLoop(
                manager=manager,
                airplane_id=aid,
                loop=loop,
                stop_event=stop_event,
                coordinator=coordinator,
                seed_waypoints=seeds,
            )
        )

    flush_task = asyncio.create_task(manager_flush_pump(manager, stop_event))
    worker_tasks = [asyncio.create_task(w.run()) for w in workers]

    mission_seconds = 90
    try:
        await asyncio.sleep(mission_seconds)
    finally:
        # 先通知各任务停止，再等待协程收尾，最后销毁底层管理器。
        stop_event.set()
        await asyncio.gather(*worker_tasks, return_exceptions=True)
        await asyncio.gather(flush_task, return_exceptions=True)
        manager.destroy()


if __name__ == "__main__":
    asyncio.run(main_async())
