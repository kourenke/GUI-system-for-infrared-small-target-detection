from time import time
from tempfile import _TemporaryFileWrapper

import cv2

from gui.utils.session import SessionState
from gui.utils.temp_file import generate_temp_file, delete_temp_file


def start_recording() -> None:
    """开始录制视频流"""
    temp_file = SessionState.get_var("temp_record_file")
    if temp_file is not None and isinstance(temp_file, _TemporaryFileWrapper):
        delete_temp_file(temp_file)
    temp_file = generate_temp_file(".avi")

    base_size = SessionState.get_var("base_size", default=256)
    write_capture = cv2.VideoWriter(
        temp_file.name,
        cv2.VideoWriter_fourcc(*"MJPG"),
        float(SessionState.get_var("record_fps")),
        (
            base_size * 3,
            base_size,
        ),
    )

    SessionState.set_var(
        is_recording=True,
        temp_record_file=temp_file,
        write_capture=write_capture,
        record_start_stmp=int(time()),
    )


def stop_recording() -> None:
    """停止录制视频流"""
    write_capture = SessionState.get_var("write_capture", default=None)

    if write_capture or write_capture.isOpened():
        write_capture.release()

    SessionState.set_var(
        has_record=True,
        is_recording=False,
        write_capture=None,
        record_stop_stmp=int(time()),
    )


def clear_record() -> None:
    """清理录制的视频流"""
    temp_file = SessionState.get_var("temp_record_file", default=None)

    if temp_file and isinstance(temp_file, _TemporaryFileWrapper):
        delete_temp_file(temp_file)

    SessionState.set_var(
        has_record=False,
        is_recording=False,
        temp_record_file=None,
        record_start_stmp=None,
        record_stop_stmp=None,
    )
