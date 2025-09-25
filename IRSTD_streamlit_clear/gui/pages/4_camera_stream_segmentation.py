from typing import Callable
from uuid import uuid4
from tempfile import _TemporaryFileWrapper
from time import sleep, strftime, gmtime, time

import av
import numpy as np
import pandas as pd
import streamlit as st
from streamlit_webrtc import (
    webrtc_streamer,
    VideoHTMLAttributes,
    WebRtcStreamerContext,
)

from models import ModelType
from utils.net import get_net
from gui.utils.session import SessionState
from gui.utils.record_video import (
    start_recording,
    stop_recording,
    clear_record,
)
from gui.utils.data_keeper import DataKeeper
from gui.utils.temp_file import get_temp_file_size
from inference_LW_IRSTNet_onnx import get_inference_result_with_args
from inference_Real_time_IR_small_target_segmentation import draw_frame

MAX_FPS = 60
AVAILABLE_SAMPLING_RATE = [
    i for i in range(1, MAX_FPS + 1) if MAX_FPS % i == 0 and i >= 10
]


class CameraDetectionPage:
    def __init__(self) -> None:
        self.__CONFIG_CONTAINER = DataKeeper()
        self.__DATA_CONTAINER = DataKeeper()

    def update_config_by_widget(self, widget_key: str, var_key: str) -> None:
        """从控件中获取设置值

        Args:
            widget_key (str): 控件名称
            var_key (str): 设置变量名称
        """
        widget_value = SessionState.get_var_from_widget(widget_key)
        config_dict = {var_key: widget_value}

        SessionState.set_var(**config_dict)
        self.__CONFIG_CONTAINER.update(config_dict)

        if var_key == "selected_target":
            print(config_dict)
            print(self.__CONFIG_CONTAINER)

    def update_config_by_callback(
        self, widget_key: str, var_key: str
    ) -> Callable[[], None]:
        """更新设置参数的回调函数工厂

        Args:
            widget_key (str): 控件名称
            var_key (str): 设置变量名称

        Returns:
            Callable[Callable[None], None]: 构造的回调函数
        """

        def callback() -> None:
            self.update_config_by_widget(widget_key, var_key)

        return callback

    def show_config_form(self) -> None:
        """显示设置表单"""
        model_tab, detect_tab, display_tab, positioning_tab = st.tabs(
            [
                "Model Config",
                "Detect Config",
                "Display Config",
                "Positioning Config",
            ]
        )

        with model_tab:

            def set_model() -> None:
                selected_model = SessionState.get_var("model_type")
                assert selected_model

                __net = get_net(ModelType[selected_model])
                data_dict = {"net": __net}

                SessionState.set_var(**data_dict)
                self.__CONFIG_CONTAINER.update(data_dict)

            st.selectbox(
                "Select Model",
                ModelType.__members__,
                key="model_type",
                on_change=set_model,
            )
            set_model()

            st.number_input(
                "Base Size",
                min_value=0,
                value=256,
                key="base_size_box",
                on_change=self.update_config_by_callback("base_size", "base_size"),
            )
            self.update_config_by_widget("base_size_box", "base_size")

        with detect_tab:
            st.markdown("Output Adjustment")

            st.number_input(
                "Inference Threshold",
                min_value=SessionState.get_var("base_size") * -1,
                max_value=SessionState.get_var("base_size"),
                value=0,
                key="output_threshold_box",
                on_change=self.update_config_by_callback(
                    "output_threshold_box", "output_threshold"
                ),
            )
            self.update_config_by_widget("output_threshold_box", "output_threshold")

            st.markdown("Target Size")

            left_container, right_container = st.columns(2)
            with left_container:
                st.number_input(
                    "Min",
                    min_value=0,
                    max_value=SessionState.get_var(
                        "max_target_size",
                        default=SessionState.get_var("base_size") ** 2,
                    ),
                    value=0,
                    key="min_target_size_box",
                    on_change=self.update_config_by_callback(
                        "min_target_size_box", "min_target_size"
                    ),
                )
                self.update_config_by_widget("min_target_size_box", "min_target_size")

            with right_container:
                st.number_input(
                    "Max",
                    min_value=SessionState.get_var("min_target_size", default=0),
                    max_value=SessionState.get_var("base_size") ** 2,
                    value=SessionState.get_var("base_size") ** 2,
                    key="max_target_size_box",
                    on_change=self.update_config_by_callback(
                        "max_target_size_box", "max_target_size"
                    ),
                )
                self.update_config_by_widget("max_target_size_box", "max_target_size")

            st.markdown("Target Aspect Ratio")

            left_container, right_container = st.columns(2)
            with left_container:
                st.number_input(
                    "Min",
                    min_value=0.0,
                    max_value=SessionState.get_var(
                        "max_target_aspect_ratio", default=1.0
                    ),
                    value=0.20,
                    key="min_target_aspect_ratio_box",
                    on_change=self.update_config_by_callback(
                        "min_target_aspect_ratio_box", "min_target_aspect_ratio"
                    ),
                )
                self.update_config_by_widget(
                    "min_target_aspect_ratio_box", "min_target_aspect_ratio"
                )

            with right_container:
                st.number_input(
                    "Max",
                    min_value=SessionState.get_var(
                        "min_target_aspect_ratio", default=1.0
                    ),
                    max_value=10.0,
                    value=5.00,
                    key="max_target_aspect_ratio_box",
                    on_change=self.update_config_by_callback(
                        "max_target_aspect_ratio_box", "max_target_aspect_ratio"
                    ),
                )
                self.update_config_by_widget(
                    "max_target_aspect_ratio_box", "max_target_aspect_ratio"
                )

        with display_tab:
            st.select_slider(
                "Data Sampling Rate",
                help="""
                    The number of inferred data updates per second.

                    > **Note:**
                    > Increasing this value may cause the video frame rate to drop.""",
                options=AVAILABLE_SAMPLING_RATE,
                value=AVAILABLE_SAMPLING_RATE[len(AVAILABLE_SAMPLING_RATE) // 2],
                key="data_sampling_rate_box",
                on_change=self.update_config_by_callback(
                    "data_sampling_rate_box", "data_sampling_rate"
                ),
            )
            self.update_config_by_widget(
                "data_sampling_rate_box",
                "data_sampling_rate",
            )

            st.slider(
                "Target selectbox update frequency",
                help="""
                    The update frequency of the data in the target selection box (second)

                    We highly recommend setting this value to a value greater than 3.
                    
                    > **Note:**
                    > If the time of the choice is too long, the data refresh is too slow, \
                        and if the time is short, it may be interrupted before choosing""",
                min_value=1,
                max_value=30,
                value=5,
                key="target_selectbox_update_frequency_box",
                on_change=self.update_config_by_callback(
                    "target_selectbox_update_frequency_box",
                    "target_selectbox_update_frequency",
                ),
            )
            self.update_config_by_widget(
                "target_selectbox_update_frequency_box",
                "target_selectbox_update_frequency",
            )

        with positioning_tab:
            st.selectbox("Positioning system", ["GPS", "Beidou"])

    def video_frame_callback(self, frame: av.VideoFrame) -> av.VideoFrame:
        """生成新的视频帧的回调函数

        Args:
            frame (av.VideoFrame): 视频帧

        Returns:
            av.VideoFrame: 生成后的新视频帧
        """
        input_image = frame.to_ndarray(format="rgb24")

        time, result, original_image, result_img = get_inference_result_with_args(
            self.__CONFIG_CONTAINER.get("net"),
            input_image,
            self.__CONFIG_CONTAINER.get("base_size", 256),
            self.__CONFIG_CONTAINER.get("output_threshold", 0),
            self.__CONFIG_CONTAINER.get("min_target_size", 0),
            self.__CONFIG_CONTAINER.get("max_target_size", 256),
            self.__CONFIG_CONTAINER.get("min_target_aspect_ratio", 0),
            self.__CONFIG_CONTAINER.get("max_target_aspect_ratio", 1.0),
        )

        original_image = (original_image * 255).astype(np.uint8)
        new_frame = draw_frame(
            original_image,
            result_img,
            result,
            self.__CONFIG_CONTAINER.get("selected_target", None),
        )

        self.__DATA_CONTAINER.update(
            {
                "time": time,
                "infer_result": result,
                "frame": new_frame,
            }
        )

        return av.VideoFrame.from_ndarray(new_frame, format="rgb24")

    def show_remote_video_stream(self) -> WebRtcStreamerContext:
        """显示处理后的视频流

        Returns:
            WebRtcStreamerContext: 远程视频流上下文
        """
        context = webrtc_streamer(
            key="video",
            video_frame_callback=self.video_frame_callback,
            media_stream_constraints={
                "video": True,
                "audio": False,
            },
            sendback_audio=False,
            async_transform=True,
            video_html_attrs=VideoHTMLAttributes(
                autoPlay=True,
                controls=True,
                style={"width": "100%", "height": "260px"},
            ),
        )

        return context

    def show_target_select_box(self) -> None:
        """显示目标选择框"""
        key_name = str(uuid4())

        target_number = self.__DATA_CONTAINER.get("infer_result", [1])[0]  # type: ignore
        select_options = list(range(1, target_number)) if target_number > 1 else []
        select_options = [None, *select_options]
        default_index = SessionState.get_var("selected_target", default=0)

        if default_index >= len(select_options):
            default_index = 0

        st.selectbox(
            label="Select target",
            options=select_options,
            index=default_index,
            format_func=lambda x: f"Target {x}",
            key=key_name,
            on_change=SessionState.set_var_from_widget_callback(
                key_name,
                "selected_target",
            ),
        )

        self.__CONFIG_CONTAINER.set(
            "selected_target",
            SessionState.get_var("selected_target"),
        )

    def show_infer_detail_tab(self) -> None:
        """显示推断详细信息选项卡"""
        infer_result = self.__DATA_CONTAINER.get("infer_result", [])
        infer_time = self.__DATA_CONTAINER.get("time", 0)

        if len(infer_result) != 4:
            return

        num_labels, _, stats, centroids = infer_result  # type: ignore
        col1, col2, col3 = st.columns(3)
        col1.metric("Number of targets", num_labels - 1)
        col2.metric("Inference time", round(infer_time, 4))
        col3.metric("Inference FPS", round(1 / infer_time, 4))

        result_table = pd.DataFrame(
            {
                "Centroid_X": centroids[1:, 0],
                "Centroid_Y": centroids[1:, 1],
                "Aspect ratio": stats[1:, 2] / stats[1:, 3],
                "Pixel": stats[1:, 4],
            }
        )

        result_table.index = np.arange(1, len(result_table) + 1)
        st.dataframe(result_table, height=200, use_container_width=True)

    def show_attitude_tab(self) -> None:
        """显示姿态选项卡"""
        col1, col2, col3 = st.columns(3)

        col1.metric("Azimuth Angle X", "Unknown")
        col2.metric("Pitch Angle Y", "Unknown")
        col3.metric("Roll Angle Z", "Unknown")

        col1.metric("Angular Velocity X", "Unknown")
        col2.metric("Angular Velocity Y", "Unknown")
        col3.metric("Angular Velocity Z", "Unknown")

        col1.metric("Acceleration X", "Unknown")
        col2.metric("Acceleration Y", "Unknown")
        col3.metric("Acceleration Z", "Unknown")

        col1.metric("Temperature", "Unknown")
        col2.metric("Air Pressure", "Unknown")
        col3.metric("Attitude", "Unknown")

    def show_positioning_tab(self) -> None:
        """显示位置标签页"""
        col1, col2, col3, col4 = st.columns(4)

        col1.metric("Ground speed", "Unknown")
        col2.metric("Surface course", "Unknown")
        col3.metric("Magnetic declination", "Unknown")
        col4.metric("Magnetic azimuth", "Unknown")

        col1.metric("Northern latitude", "Unknown")
        col2.metric("Southern latitude", "Unknown")
        col3.metric("West meridian", "Unknown")
        col4.metric("East longitude", "Unknown")

    def show_record_config_form(self) -> None:
        """显示录制控制项"""
        st.selectbox(
            "Record FPS",
            AVAILABLE_SAMPLING_RATE,
            key="record_fps_box",
            disabled=SessionState.get_var("is_recording", default=False),
            on_change=SessionState.set_var_from_widget_callback(
                "record_fps_box",
                "record_fps",
            ),
        )
        SessionState.set_var_from_widget(
            "record_fps_box",
            "record_fps",
        )

    def show_record_info(self) -> None:
        """显示录制的视频信息"""
        col1, col2 = st.columns(2)

        with col1:
            start_stmp = SessionState.get_var("record_start_stmp")
            stop_stmp = SessionState.get_var("record_stop_stmp")

            if not stop_stmp:
                duration_sec = 0
            elif not stop_stmp:
                duration_sec = int(time()) - start_stmp
            else:
                duration_sec = stop_stmp - start_stmp

            st.metric(
                "Recording Duration",
                strftime("%H:%M:%S", gmtime(duration_sec)),
            )

        with col2:
            temp_file = SessionState.get_var("temp_record_file")

            if temp_file and isinstance(temp_file, _TemporaryFileWrapper):
                file_size = get_temp_file_size(temp_file)
            else:
                file_size = "Unknown"

            st.metric("Record File Size", file_size)

    def show_record_control_button(self) -> None:
        """显示录制控制按"""
        col1, col2 = st.columns(2)

        control_button_key = str(uuid4())
        with col1:
            if not SessionState.get_var("is_recording", default=False):
                st.button(
                    "Start Recording",
                    key=control_button_key,
                    on_click=start_recording,
                    use_container_width=True,
                )
            else:
                st.button(
                    "Stop Recording",
                    key=control_button_key,
                    on_click=stop_recording,
                    use_container_width=True,
                )

        download_key = str(uuid4())
        with col2:
            if (
                not SessionState.get_var("has_record", default=False)
            ) or SessionState.get_var("is_recording", default=False):
                st.button(
                    "Download Record",
                    key=download_key,
                    disabled=True,
                    use_container_width=True,
                )
            else:
                download_file = SessionState.get_var("temp_record_file")
                assert isinstance(download_file, _TemporaryFileWrapper)

                st.download_button(
                    "Download Record",
                    download_file.read(),
                    key=download_key,
                    file_name="infer_record.avi",
                    on_click=clear_record,
                    use_container_width=True,
                )

    def record_frame_to_capture(self) -> None:
        """录制视频帧"""
        frame = self.__DATA_CONTAINER.get("frame", None)
        write_capture = SessionState.get_var("write_capture", default=None)

        if frame is not None and write_capture and write_capture.isOpened():
            write_capture.write(frame)

    def show_page(self) -> None:
        """显示页面"""
        st.title("Camera Stream Detection")

        st.markdown(
            """Description: This app detects the camera stream from the webcam."""
        )

        self.show_config_form()

        stream_context = self.show_remote_video_stream()

        if not stream_context.state.playing:
            st.stop()

        with st.container():
            seletbox_placeholder = st.empty()

            infer_detail_tab, attitude_tab, positioning_tab, record_tab = st.tabs(
                ["Inference Details", "Attitude", "Positioning", "Record"]
            )

            infer_detail_tab = infer_detail_tab.empty()
            attitude_tab = attitude_tab.empty()
            positioning_tab = positioning_tab.empty()

            with record_tab:
                self.show_record_config_form()
                record_info_container = st.empty()
                self.show_record_control_button()

        # 时间标记
        frame_marker = 0
        second_marker = 0

        # 数据更新循环
        while stream_context.state.playing:
            update_fequency = self.__CONFIG_CONTAINER.get(
                "target_selectbox_update_frequency",
                5,
            )
            data_sampling_rate = self.__CONFIG_CONTAINER.get(
                "data_sampling_rate",
                10,
            )
            is_recording = SessionState.get_var("is_recording", default=False)

            if second_marker == 0:
                with seletbox_placeholder.container():
                    self.show_target_select_box()

            if frame_marker == 0 or frame_marker % data_sampling_rate == 0:
                with infer_detail_tab.container():
                    self.show_infer_detail_tab()

                with attitude_tab.container():
                    self.show_attitude_tab()

                with positioning_tab.container():
                    self.show_positioning_tab()

            if is_recording and (
                frame_marker == 0
                or frame_marker % SessionState.get_var("record_fps", default=MAX_FPS)
            ):
                self.record_frame_to_capture()

            with record_info_container.container():
                self.show_record_info()

            if frame_marker == (MAX_FPS - 1):
                frame_marker = 0

                if second_marker == (update_fequency - 1):
                    second_marker = 0
                else:
                    second_marker += 1
            else:
                frame_marker += 1

            sleep(1 / MAX_FPS)

        clear_record()


page_object = CameraDetectionPage()
page_object.show_page()
