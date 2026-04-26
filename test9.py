import asyncio
import time

from uav import UAVAirplaneManager, get_airplane_manager
from uav.FH0C.image_process import write_mat_2_file

# 图像回调处理
def handle_image(airplane_id, img):
    print(f"[{airplane_id}] Async image received: {len(img)} bytes")
    time_str = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
    write_mat_2_file(img, f"image_{airplane_id.replace(':', '-')}_{time_str}.jpg")


# 异步飞行任务
async def async_flight_mission(manager, airplane_id):
    a = manager.get_airplane(airplane_id)

    # a.use_fast_mode(
    #     fast_mode=False,
    #     future_mode=True,
    # )

    a.mode(4)
    await asyncio.sleep(2)

    a.takeoff(100)
    await asyncio.sleep(5)  # 让出 CPU 控制权给其他飞机

    # a.left(100)
    # await asyncio.sleep(2)

    # 触发抓图（底层必须是非阻塞的）
    a.cap_image(
        user_receive_callback=lambda img: handle_image(airplane_id, img),
        user_progress_callback=lambda p, t: None
    )

    # 继续飞行，互不干扰
    a.up(50)
    await asyncio.sleep(2)

    manager.flush()
    # print(a.status)
    # print(a.get_state())
    # print(a.status.x, a.status.y, a.status.h)

    # a.goto(a.status.x, 300, 300)
    # await asyncio.sleep(3)

    a.land()
    await asyncio.sleep(5)
    print(f"[{airplane_id}] Landed.")


async def main_async():
    m = get_airplane_manager()
    m.flush()
    m.start()
    m.flush()

    airplane_ids = [
        # 'COM3',
        # 'COM4',
        # 'COM5',
        'FH0C:COM3',
        # 'FH0C:COM4',
        # 'FH0C:COM5',
    ]

    # 并发执行所有飞机的任务
    tasks = [async_flight_mission(m, aid) for aid in airplane_ids]
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    asyncio.run(main_async())
