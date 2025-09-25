import pandas as pd
import streamlit as st

from models import ModelType
from gui.utils.session import SessionState
from gui.utils.widget import (
    show_inference_result,
    show_download_button_of_dataframe,
)
from utils.net import get_net_by_device
from utils.image_utils import convert_to_opencv_image


def show_config_form() -> bool:
    """显示设置表单

    Returns:
        bool: 是否提交表单
    """
    with st.form("detect_config_form"):
        # selected_model = st.selectbox(
        #     "Select Model",
        #     ModelType.__members__,
        #     key="model_type",
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

        st.file_uploader(
            "Select an image",
            type=[
                "jpg",
                "png",
            ],
            accept_multiple_files=False,
            key="upload_image_box",
        )
        SessionState.set_var_from_widget(
            "upload_image_box",
            "uploaded_image",
            callback=lambda x: x if x is None else convert_to_opencv_image(x),
        )

        with st.expander(label="More Configs", expanded=False):
            st.number_input(
                "Base Size",
                min_value=0,
                value=256,
                key="base_size_box",
            )
            SessionState.set_var_from_widget("base_size_box", "base_size")

        submitted = st.form_submit_button(
            "Detect",
            type="primary",
            use_container_width=True,
        )

    return submitted


def show_download_button(infer_result: pd.DataFrame) -> None:
    """显示下载按钮

    Args:
        infer_result (pd.DataFrame): 推断结果数据
    """
    show_download_button_of_dataframe(
        "Download inference result",
        infer_result,
        file_name="infer_result.csv",
    )


def show_page() -> None:
    """显示页面"""
    st.title("Single Frame Segmentation")

    st.markdown(
        """
        Here are some examples of how to use the Single Frame Detection model.
        """
    )

    is_submit = show_config_form()

    if not is_submit:
        st.stop()

    if SessionState.get_var("uploaded_image") is None:
        st.error("Please upload an image.")
        st.stop()

    net = get_net_by_device(SessionState.get_var('selected_model').value)
    _, result_data = show_inference_result(
        net,
        uploaded_image=SessionState.get_var("uploaded_image"),
        base_size=SessionState.get_var("base_size"),
    )

    show_download_button(result_data)


show_page()
