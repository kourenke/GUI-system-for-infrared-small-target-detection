from tempfile import _TemporaryFileWrapper

import cv2
import streamlit as st
from streamlit.runtime.uploaded_file_manager import UploadedFile

from models import ModelType
from utils.net import get_net_by_device
from utils.image_utils import convert_to_rgb_image
from gui.utils.session import SessionState
from gui.utils.widget import show_inference_result
from gui.utils.temp_file import (
    delete_temp_file,
    generate_temp_file,
)
from inference_opencv_video import get_video_info
from inference_LW_IRSTNet_onnx import get_inference_result


def close_capture(var_key: str) -> None:
    """关闭正在使用的 VideoCapture 对象

    Args:
        var_key (str): 存储使用的 VideoCapture 对象的变量名
    """
    capture = SessionState.get_var(var_key)

    if capture is not None:
        capture.release()


def load_to_temp_file(uploaded_file: UploadedFile) -> _TemporaryFileWrapper:
    """创建临时文件用于存储上传的视频

    Args:
        uploaded_file (UploadedFile): 上传的文件对象

    Returns:
        _TemporaryFileWrapper: 临时文件对象
    """
    temp_file = generate_temp_file()
    temp_file.write(uploaded_file.read())
    return temp_file


def init_inference_config() -> None:
    """初始化推断参数"""
    close_capture("read_capture")
    delete_temp_file(temp_file=SessionState.get_var("uploaded_file"))
    SessionState.set_var(
        frame_index=0,
        is_submitted=True,
        uploaded_file=None,
    )


def clear_inference_config() -> None:
    """清空推断参数"""

    try:
        close_capture("read_capture")
        delete_temp_file(temp_file=SessionState.get_var("uploaded_file"))
        close_capture("write_capture")
        delete_temp_file(temp_file=SessionState.get_var("save_temp_file"))
    finally:
        SessionState.set_var(
            frame_index=None,
            is_submitted=False,
            uploaded_file=None,
            save_temp_file=None,
            read_capture=None,
            write_capture=None,
        )


def show_config_form() -> bool:
    """显示设置表单

    Returns:
        bool: 是否提交表单
    """
    with st.form("detect_config_form"):
        # selected_model = st.selectbox(
        #     "Select Model",
        #     ModelType.__members__,
        # )
        # assert selected_model
        # SessionState.set_var(net=get_net(ModelType[selected_model]))

        st.selectbox(
            "Select Model",
            ModelType.__members__,
            key="model_box",
        )
        SessionState.set_var_from_widget(
            "model_box",
            "selected_model",
            callback=lambda x: ModelType[x],
        )

        __uploaded_file = st.file_uploader(
            "Select an video",
            type=[
                "avi",
                "mp4",
            ],
            accept_multiple_files=False,
        )
        if __uploaded_file is not None:
            delete_temp_file(temp_file=SessionState.get_var("uploaded_file"))
            SessionState.set_var(uploaded_file=load_to_temp_file(__uploaded_file))

        with st.expander(label="More Configs", expanded=False):
            __base_size = st.number_input(
                "Base Size",
                min_value=0,
                value=256,
                key="base_size_box",
            )
            if __base_size is not None:
                SessionState.set_var(base_size=__base_size)

        submitted = st.form_submit_button(
            "Detect",
            type="primary",
            use_container_width=True,
            on_click=init_inference_config,
        )

    return submitted


def show_clear_page_button() -> None:
    """显示清空页面按钮"""
    st.button(
        "Clear Page",
        on_click=clear_inference_config,
        use_container_width=True,
    )


def generate_video_to_save(
    base_size: int,
    read_capture: cv2.VideoCapture,
    write_capture: cv2.VideoCapture,
) -> None:
    """向临时视频文件中写入处理后的帧

    Args:
        base_size (int): 基础大小
        read_capture (cv2.VideoCapture): 上传的视频对象
        write_capture (cv2.VideoCapture): 用于写入的视频对象
    """
    # network = SessionState.get_var("net")
    print(f"value = {SessionState.get_var('selected_model').value}")
    print(f"name = {SessionState.get_var('selected_model').name}")
    network = get_net_by_device(SessionState.get_var('selected_model').value)

    read_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)

    while read_capture.isOpened():
        success, frame = read_capture.read()

        if not success:
            break

        *_, result_frame = get_inference_result(network, frame, base_size)
        result_frame = convert_to_rgb_image(result_frame)

        write_capture.write(result_frame)


def show_download_button() -> None:
    """显示下载按钮占位符（不可用）"""
    need_download = st.button(
        "Need Download Video",
        use_container_width=True,
    )

    if need_download:
        temp_file = SessionState.get_var("uploaded_file")
        assert isinstance(temp_file, _TemporaryFileWrapper)

        capture_to_read = cv2.VideoCapture(temp_file.name)

        if not capture_to_read.isOpened():
            st.error("Error opening video stream or file.")
            st.stop()

        update_download_button(capture_to_read)

        capture_to_read.release()


def update_download_button(read_capture: cv2.VideoCapture) -> None:
    """更新下载处理后视频的按钮

    Args:
        read_capture (cv2.VideoCapture): 上传的视频对象
    """
    base_size = SessionState.get_var("base_size")
    assert isinstance(base_size, int)

    temp_file = generate_temp_file(".avi")
    SessionState.set_var(save_temp_file=temp_file)

    *__size, __total_frames, fps = get_video_info(read_capture)

    write_capture = cv2.VideoWriter(
        temp_file.name,
        cv2.VideoWriter_fourcc(*"MJPG"),
        fps,
        (base_size, base_size),
    )
    SessionState.set_var(write_capture=write_capture)

    generate_video_to_save(base_size, read_capture, write_capture)

    st.download_button(
        label="Download Video",
        data=temp_file.read(),
        file_name="infer_result.avi",
        use_container_width=True,
    )


def show_frame_inference_result(capture: cv2.VideoCapture) -> None:
    """显示视频推断结果

    Args:
        capture (cv2.VideoCapture): 视频对象
    """
    # network = SessionState.get_var("net")
    network = get_net_by_device(SessionState.get_var('selected_model').value)
    base_size = SessionState.get_var("base_size")
    assert isinstance(base_size, int)

    __frame_width, __frame_height, total_frames, __fps = get_video_info(capture)

    progress_bar_placeholder = st.empty()
    inference_result_placeholder = st.empty()

    while capture.isOpened():
        success, frame = capture.read()

        if not success:
            with progress_bar_placeholder:
                st.slider("Frame Index", value=1.0)
            break

        frame_index = SessionState.get_var("frame_index")
        assert isinstance(frame_index, int)

        slider_key = f"frame_{frame_index}"
        with progress_bar_placeholder:
            st.slider(
                "Frame Index",
                key=slider_key,
                min_value=0.0,
                max_value=1.0,
                value=(frame_index / (total_frames - 1)),
                on_change=SessionState.set_var_from_widget_callback(
                    widget_key=slider_key,
                    var_key="frame_index",
                    callback=lambda x: int(x * (total_frames - 1)),
                ),
                disabled=True,
            )

        with inference_result_placeholder:
            with st.container():
                show_inference_result(network, frame, base_size=base_size)

        SessionState.set_var(frame_index=frame_index + 1)


def show_page() -> None:
    """显示页面"""
    st.title("Video Segmentation")

    st.markdown(
        """
        Here are some descriptions about the video detection model.
        """
    )

    show_config_form()

    if not SessionState.get_var("is_submitted"):
        st.stop()

    if SessionState.get_var("uploaded_file") is None:
        st.error("Please upload an Video.")
        st.stop()

    show_clear_page_button()

    with st.empty():
        show_download_button()

    temp_file = SessionState.get_var("uploaded_file")
    assert isinstance(temp_file, _TemporaryFileWrapper)


    read_capture = cv2.VideoCapture(temp_file.name)
    read_capture.set(
        cv2.CAP_PROP_POS_FRAMES,
        SessionState.get_var("frame_index", default=0),
    )
    SessionState.set_var(read_capture=read_capture)

    if not read_capture.isOpened():
        st.error("Error opening video stream or file.")
        st.stop()

    with st.container():
        show_frame_inference_result(read_capture)

    read_capture.release()

    init_inference_config()


show_page()
