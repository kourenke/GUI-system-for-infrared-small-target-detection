import cv2
import torch
from streamlit import cache_resource

from models import ModelType


@cache_resource(show_spinner=False)
def get_net(model: ModelType):
    """获取 net 对象"""
    return cv2.dnn.readNetFromONNX(model.value)


@cache_resource(show_spinner=False)
def get_device() -> torch.device:
    """获取 device 对象"""
    return torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


@cache_resource(show_spinner=False)
def get_net_by_device(model_path:str, device: torch.device = 'cpu'):
    """load network

    Args:
        device (torch.device): _description_

    Returns:
        _type_: network
    """
    # pkl_path = r"merged_LW_IRST_ablation_Iter-18400_mIoU-0.6638_fmeasure-0.7979.pkl"
    if model_path.endswith('pkl'):
        net = torch.load(model_path, map_location=device)
        print("***"*20)
        print(f"Use = {model_path}")
        return net.eval()
    elif model_path.endswith('onnx'):
        print(model_path)
        return cv2.dnn.readNetFromONNX(model_path) #TypeError: 'cv2.dnn.Net' object is not callable?????
    else:
        raise ValueError(f"Not support file format{model_path}")

