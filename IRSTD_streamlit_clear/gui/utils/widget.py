from typing import Optional, Tuple, Any

import numpy as np
import pandas as pd
import streamlit as st

from utils.image_utils import convert_to_rgb_image
from inference_LW_IRSTNet_onnx import get_inference_result


def show_compare_images(
    original_image: np.ndarray,
    result_image: np.ndarray,
) -> None:
    """显示对比图片

    Args:
        original_image (np.ndarray): 原图
        result_image (np.ndarray): 结果图
    """
    left_container, right_container = st.columns(2)

    with left_container:
        st.image(
            original_image,
            caption="Original Image",
            use_column_width=True,
        )

    with right_container:
        st.image(
            result_image,
            caption="Result Image",
            use_column_width=True,
        )


def show_download_button_of_dataframe(
    label: str,
    data: pd.DataFrame,
    *,
    file_name: Optional[str] = None,
) -> None:
    """显示下载按钮

    Args:
        label (str): 按钮上要显示的文本
        data (pd.DataFrame): 要下载的数据
        file_name (Optional[str], optional): 文件名（需要以 .csv 结尾）. Defaults to None.
    """
    st.download_button(
        label=label,
        data=data.to_csv().encode("utf-8"),
        file_name=file_name or "data.csv",
        mime="text/csv",
        use_container_width=True,
    )


def show_inference_result(
    __net: Any,
    uploaded_image: np.float32,
    base_size: Optional[int] = None,
) -> Tuple[np.ndarray, pd.DataFrame]:
    """显示推断结果

    Args:
        __net (Any): network
        uploaded_image (np.float32): 要推断的图片
        base_size (Optional[int], optional): base_size. Defaults to 256.

    Returns:
        Tuple[np.ndarray, pd.DataFrame]: 推断结果图片，推断结果数据
    """
    infer_time, infer_result, original_img, mask_img = get_inference_result(
        __net,
        image=uploaded_image,
        base_size=base_size or 256,
    )
    num_labels, _, stats, centroids = infer_result

    col1, col2, col3 = st.columns(3)
    col1.metric("Number of targets", num_labels - 1)
    col2.metric("Inference time", round(infer_time, 4))
    col3.metric("Inference FPS", round(1 / infer_time, 4))

    mask_img = convert_to_rgb_image(mask_img)
    show_compare_images(original_img, mask_img)

    result_table = pd.DataFrame(
        {
            "Centroid_X": centroids[1:, 0],
            "Centroid_Y": centroids[1:, 1],
            "Aspect ratio": stats[1:, 2] / stats[1:, 3],
            "Pixel": stats[1:, 4],
        }
    )
    # result_table["Centroid"] = result_table[["Centroid_X", "Centroid_Y"]].apply(
    #     lambda x: f"( {x['Centroid_X']:.2f}, {x['Centroid_Y']:.2f} )",
    #     axis=1,
    # )
    result_table.index = np.arange(1, len(result_table) + 1)
    st.dataframe(result_table, height=200, use_container_width=True)

    return mask_img, result_table
