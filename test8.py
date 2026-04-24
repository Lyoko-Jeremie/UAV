from __future__ import annotations

from datetime import datetime
from pathlib import Path
from threading import Event
from time import sleep
from typing import Any

from uav import UAVAirplaneManager, get_airplane_manager


DRONE_ID = "FH0C:COM3"
TAKEOFF_H = 100


def wait_previous_transfer_done(airplane: Any, poll_s: float = 0.1) -> None:
    # Must wait for previous image transfer to finish before next capture.
    while airplane.image_receiver.is_transfer_in_progress():
        sleep(poll_s)


def capture_image(airplane: Any) -> tuple[Event, dict[str, bytes]]:
    """
    Initiate async image capture without blocking.
    Stabilize drone before calling this function.
    Returns (done_event, image_box) for later retrieval.
    """
    wait_previous_transfer_done(airplane)

    done = Event()
    image_box: dict[str, bytes] = {}

    def on_receive(image_bytes: bytes) -> None:
        image_box["data"] = image_bytes
        done.set()

    order_count = airplane.cap_image(
        user_receive_callback=on_receive,
        user_progress_callback=lambda _cur, _total: None,
    )
    if order_count is None:
        raise RuntimeError("Capture request rejected: previous transfer may still be running.")

    # Return immediately - image transfer happens in background
    return done, image_box


def wait_and_save_image(
    done: Event, image_box: dict[str, bytes], airplane: Any, timeout_s: float = 30.0
) -> Path:
    """Wait for image transfer to complete and save to file."""
    if not done.wait(timeout=timeout_s):
        raise TimeoutError(f"Image transfer timeout (>{timeout_s:.0f}s)")

    # Re-check transfer state after callback to avoid race on immediate next capture.
    wait_previous_transfer_done(airplane)

    image_data = image_box.get("data")
    if not image_data:
        raise RuntimeError("Image callback fired but no image data was received.")

    file_name = datetime.now().strftime("%Y%m%d_%H%M%S_%f.jpg")
    file_path = Path.cwd() / file_name
    file_path.write_bytes(image_data)
    print(f"saved: {file_path.name}")
    return file_path


def main() -> None:
    manager: UAVAirplaneManager = get_airplane_manager()
    airplane = None
    has_taken_off = False
    pending_captures: list[tuple[Event, dict[str, bytes]]] = []

    try:
        print(manager.start())
        airplane = manager.get_airplane_extended(DRONE_ID)
        if airplane is None:
            raise RuntimeError(f"Cannot get airplane object: {DRONE_ID}")

        # For FH0C, goto should run in mode 1.
        if hasattr(airplane, "mode"):
            airplane.mode(1)
            sleep(1)

        airplane.takeoff(TAKEOFF_H)
        has_taken_off = True
        sleep(5)

        # First waypoint and capture
        airplane.goto(120, 120, TAKEOFF_H)
        sleep(1)  # Stabilize ~1s before capture

        pending_captures.append(capture_image(airplane))  # Initiate capture
        sleep(1)  # Stabilize ~1s after capture

        # Fly to next waypoints - image transfer happens in background
        airplane.goto(220, 120, TAKEOFF_H)
        sleep(4)
        airplane.goto(220, 220, TAKEOFF_H)
        sleep(1)  # Stabilize ~1s before capture

        pending_captures.append(capture_image(airplane))  # Initiate capture
        sleep(1)  # Stabilize ~1s after capture

        airplane.goto(120, 220, TAKEOFF_H)
        sleep(4)

    finally:
        if airplane is not None and has_taken_off:
            airplane.land()
            sleep(5)

        # Wait for and save all pending image transfers
        for i, (done, image_box) in enumerate(pending_captures):
            try:
                wait_and_save_image(done, image_box, airplane)
            except Exception as e:
                print(f"Error saving image {i + 1}: {e}")

        if airplane is not None:
            airplane.shutdown()
        manager.destroy()


if __name__ == "__main__":
    main()
