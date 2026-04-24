import asyncio
from dataclasses import dataclass
from typing import Any, List

import cv2
import numpy as np

from uav import UAVAirplaneManager, get_airplane_manager


@dataclass
class Waypoint:
    x: int
    y: int
    h: int


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
    def __init__(
        self,
        manager: UAVAirplaneManager,
        airplane_id: str,
        loop: asyncio.AbstractEventLoop,
        stop_event: asyncio.Event,
        takeoff_h: int = 120,
        capture_interval_s: float = 1.0,
    ):
        self.manager = manager
        self.airplane_id = airplane_id
        self.loop = loop
        self.stop_event = stop_event
        self.takeoff_h = takeoff_h
        self.capture_interval_s = capture_interval_s

        self.airplane: Any = manager.get_airplane(airplane_id)
        self.image_queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=2)
        self.route_queue: asyncio.Queue[Waypoint] = asyncio.Queue(maxsize=16)
        self.last_waypoint = Waypoint(0, 0, takeoff_h)
        self.has_taken_off = False

    def _on_image_received(self, image_bytes: bytes) -> None:
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
        while not self.stop_event.is_set():
            try:
                wp = await asyncio.wait_for(self.route_queue.get(), timeout=0.5)
            except asyncio.TimeoutError:
                continue

            self.airplane.goto(int(wp.x), int(wp.y), int(wp.h))
            self.last_waypoint = wp
            await asyncio.sleep(0.35)

    async def _capture_loop(self) -> None:
        while not self.stop_event.is_set():
            self.airplane.cap_image(
                user_receive_callback=self._on_image_received,
                user_progress_callback=lambda _progress, _total: None,
            )
            await asyncio.sleep(self.capture_interval_s)

    async def _plan_loop(self) -> None:
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
                if self.route_queue.full():
                    try:
                        self.route_queue.get_nowait()
                    except asyncio.QueueEmpty:
                        pass
                self.route_queue.put_nowait(wp)

    async def run(self) -> None:
        tasks: List[asyncio.Task[Any]] = []
        try:
            self.airplane.takeoff(self.takeoff_h)
            self.has_taken_off = True
            await asyncio.sleep(4)

            # 起飞后先投递首批航点，尽快进入飞行-感知-重规划循环。
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
    while not stop_event.is_set():
        manager.flush()
        await asyncio.sleep(0.2)


async def main_async() -> None:
    manager = get_airplane_manager()
    manager.start()
    manager.flush()

    airplane_ids = [
        "FH0C:COM3",
        "FH0C:COM4",
        "FH0C:COM5",
    ]

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()
    workers = [AsyncDroneFeedbackLoop(manager, aid, loop, stop_event) for aid in airplane_ids]

    flush_task = asyncio.create_task(manager_flush_pump(manager, stop_event))
    worker_tasks = [asyncio.create_task(w.run()) for w in workers]

    mission_seconds = 90
    try:
        await asyncio.sleep(mission_seconds)
    finally:
        stop_event.set()
        await asyncio.gather(*worker_tasks, return_exceptions=True)
        await asyncio.gather(flush_task, return_exceptions=True)
        manager.destroy()


if __name__ == "__main__":
    asyncio.run(main_async())
