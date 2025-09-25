from enum import Enum
import os
import os.path as osp
from .LW_IRST_Net.LW_IRST_ablation import *
# from .DNANet.model_DNANet import *
# from .MTUNet.MTU_Net import *
# from .ABC.ABCNet import *

# __all__ = ["LW_IRST_ablation"]


# class ModelType(Enum):
#     LW_IRST_ablation = "./LW_IRST.onnx"


CKPT_ROOT_PATH = './checkpoint'
ckpt_list = os.listdir(CKPT_ROOT_PATH)
ModelType = Enum('ModelType', {name:osp.join(CKPT_ROOT_PATH,name) for name in ckpt_list})



def get_segmentation_model(name):
    if name == "LW_IRST_ablation":
        net = LW_IRST_ablation(
            channel=(8, 32, 64),
            dilations=(2, 4, 8, 16),
            kernel_size=(7, 7, 7, 7),
            padding=(3, 3, 3, 3),
        )
    else:
        raise NotImplementedError
    return net
