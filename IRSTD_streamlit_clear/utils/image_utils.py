import cv2
import torch
import numpy as np
from streamlit import cache_resource
from streamlit.runtime.uploaded_file_manager import UploadedFile


def preprocess_image(img: cv2.Mat) -> cv2.Mat:
    """图像预处理

    Args:
        img (cv2.Mat): 图像数组

    Returns:
        cv2.Mat: 处理后的图像数组
    """
    means = [0.485, 0.456, 0.406]
    stds = [0.229, 0.224, 0.225]

    # means = [0.3963, 0.3963, 0.3963]
    # stds = [0.1357, 0.1357, 0.1357]


    preprocessed_img = img.copy()[:, :, ::-1]
    for i in range(3):
        preprocessed_img[:, :, i] = preprocessed_img[:, :, i] - means[i]
        preprocessed_img[:, :, i] = preprocessed_img[:, :, i] / stds[i]
    preprocessed_img = np.ascontiguousarray(np.transpose(preprocessed_img, (2, 0, 1)))
    preprocessed_img = torch.from_numpy(preprocessed_img)
    preprocessed_img.unsqueeze_(0)
    preprocessed_img = preprocessed_img.numpy()
    # preprocessed_img = preprocessed_img[:, 0:1, :, :]
    print(preprocessed_img.shape)

    return preprocessed_img


@cache_resource(show_spinner=False, ttl=24 * 60 * 60)
def preprocess_image_with_cache(img: cv2.Mat) -> cv2.Mat:
    """图像预处理，与 preprocess_image 的区别在于使用了 streamlit 提供的缓存 api

    Args:
        img (cv2.Mat): 图像数组

    Returns:
        cv2.Mat: 处理后的图像数组
    """
    return preprocess_image(img)


def resize_image(image: cv2.Mat, base_size: int) -> cv2.Mat:
    """转换为统一的图像尺寸

    Args:
        image (cv2.Mat): 图像数组
        base_size (int): _description_

    Returns:
        cv2.Mat: 转换后的图片数组
    """
    return np.float32(cv2.resize(image, (base_size, base_size))) / 255  # type: ignore


@cache_resource(show_spinner=False, ttl=24 * 60 * 60)
def convert_to_opencv_image(image_obj: UploadedFile) -> np.ndarray:
    """将上传的文件转换为 OpenCV 的图片数值

    Args:
        image_obj (UploadedFile): 上传的图片文件

    Returns:
        Any: 转换后的图片数组
    """
    file_bytes = np.asarray(bytearray(image_obj.read()), dtype=np.uint8)
    return cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)


def convert_to_rgb_image(image_array: np.ndarray) -> np.ndarray:
    """将逻辑数组转换为 rgb 图像数组

    Args:
        image_array (np.ndarray): 逻辑数组

    Returns:
        np.float32: 转换后的图片数组
    """
    return cv2.cvtColor(
        np.uint8(image_array) * 255,
        cv2.COLOR_GRAY2BGR,
    )  # type: ignore
