from typing import Tuple, Any
import pandas as pd
import streamlit as st
from matplotlib import pyplot as plt
from models import ModelType
from utils.net import get_net
from utils.data import DatasetType
from utils.MatchType import MatchType
from utils.DisType import DisType
from gui.utils.session import SessionState
from gui.utils.widget import show_download_button_of_dataframe
from evaluation import (
    evaluate,
    get_metrics,
    get_dataloader,
    get_evaluate_info,
)



def show_config_form() -> bool:
    """显示设置表单

    Returns:
        bool: 是否提交表单
    """
    with st.form("evaluate_config_form"):
        # form_left_column, form_right_column = st.columns(2)

        st.selectbox(
            "Select Dataset",
            DatasetType.__members__,
            key="dataset_box",
        )
        SessionState.set_var_from_widget(
            "dataset_box",
            "selected_dataset",
            callback=lambda x: DatasetType[x],
        )

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

        with st.expander(label="More Configs", expanded=False):
            st.number_input(
                "Base Size",
                min_value=0,
                value=256,
                key="base_size_box",
            )
            SessionState.set_var_from_widget("base_size_box", "base_size")

            st.number_input(
                "Batch Size",
                min_value=0,
                value=16,
                key="batch_size_box",
            )
            SessionState.set_var_from_widget("batch_size_box", "batch_size")

            st.number_input(
                "Confidence threshold",
                min_value=0.,
                value=0.5,
                key="conf_thr_box",
            )
            SessionState.set_var_from_widget("conf_thr_box", "conf_thr")

            st.number_input(
                "Confidence thresholds",
                min_value=0,
                value=50,
                key="conf_thrs_box",
            )
            SessionState.set_var_from_widget("conf_thrs_box", "conf_thrs")

            st.number_input(
                "dilate kernel",
                min_value=0,
                value=0,
                key="dilate_kernel_box",
            )
            SessionState.set_var_from_widget("dilate_kernel_box", "dilate_kernel")

            st.selectbox(
                "Select dis match type",
                DisType.__members__,
                key="dis_match_box",
            )
            SessionState.set_var_from_widget(
                "dis_match_box",
                "selected_dis_match_type",
                callback=lambda x: DisType[x],
            )

            st.selectbox(
                "Select second match type",
                MatchType.__members__,
                key="second_match_box",
            )
            SessionState.set_var_from_widget(
                "second_match_box",
                "selected_second_match_type",
                callback=lambda x: MatchType[x],
            )

        submitted = st.form_submit_button(
            "Evaluate",
            type="primary",
            use_container_width=True,
        )

    return submitted


def evaluate_model() -> Tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any]:
    """评估模型

    Returns:
        Tuple[Any, Any, Any, Any, Any]: metrics, metric_roc_pr, metric_center, metric_pd_fd
    """

    PixelPrecisionRecallF1IoU, PixelROCPrecisionRecall, PixelNormalizedIoU, HybridNormalizedIoU, TargetPrecisionRecallF1, TargetAveragePrecision, TargetPdPixelFa, TargetPdPixelFaROC, mNoCoAP, FPS = get_metrics(SessionState)
    data_loader = get_dataloader(
        SessionState.get_var("selected_dataset"),
        SessionState.get_var("base_size"),
        SessionState.get_var("batch_size"),
    )
    model_path = SessionState.get_var('selected_model').value

    total_mission = len(data_loader)
    progress_bar = st.progress(value=0.0)

    for i, (data, labels) in enumerate(data_loader):
        index = i + 1
        progress_bar.progress(
            value=(index / total_mission),
            text=(
                f"Processing ( {index} / {total_mission} ), please wait..."
                + (" Success!" if index == total_mission else "")
            ),
        )
        evaluate(data, labels, PixelPrecisionRecallF1IoU, PixelROCPrecisionRecall, PixelNormalizedIoU, HybridNormalizedIoU, TargetPrecisionRecallF1, TargetAveragePrecision, TargetPdPixelFa, TargetPdPixelFaROC, mNoCoAP, FPS, model_path=model_path)

    return PixelPrecisionRecallF1IoU, PixelROCPrecisionRecall, PixelNormalizedIoU, HybridNormalizedIoU, TargetPrecisionRecallF1, TargetAveragePrecision, TargetPdPixelFa, TargetPdPixelFaROC, mNoCoAP, FPS


def show_evaluate_info() -> Tuple[pd.DataFrame, pd.DataFrame , pd.DataFrame]:
    """展示模型评估信息

    Returns:
        Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]: 评估数据的元组，包含三组数据：

            1. 包含两列分别是 key 和对应的 value
            2. 用于绘ROC图的数据
            2. 用于绘PR图的数据
    """
    container = st.empty()

    with container:
        PixelPrecisionRecallF1IoU, PixelROCPrecisionRecall, PixelNormalizedIoU, HybridNormalizedIoU, TargetPrecisionRecallF1, TargetAveragePrecision, TargetPdPixelFa, TargetPdPixelFaROC, mNoCoAP, FPS = evaluate_model()
        evaluate_info, roc_data, pr_data= get_evaluate_info(
            PixelPrecisionRecallF1IoU,
            PixelROCPrecisionRecall,
            PixelNormalizedIoU,
            HybridNormalizedIoU,
            TargetPrecisionRecallF1,
            TargetAveragePrecision,
            TargetPdPixelFa,
            TargetPdPixelFaROC,
            mNoCoAP,
            __metric_FPS = FPS,
            base_size=SessionState.get_var("base_size"),
        )

        left_container = container.columns(1)[0]

        with left_container:
            # 置信度阈值为0.5时的评价指标
            html_title = """  
            <div style="text-align:center; font-size:20px; font-weight:bold;">  
                High usage indicators (Fixed conf.Th)  
            </div>  
            """
            st.markdown(html_title, unsafe_allow_html=True)
            column_num = 4
            columns = st.columns(column_num)
            for index, (_, row) in enumerate(evaluate_info.iterrows()):
                metric_label = row.key
                metric_value = row.value

                if isinstance(metric_value, float):
                    metric_value = round(metric_value, 4)

                columns[index % column_num].metric(metric_label, metric_value)

            # 计算目标级 Precision, Recall and F1-Score
            html_title = """  
            <div style="text-align:center; font-size:20px; font-weight:bold;">  
                Target-level Precision, Recall and F1-Score (Fixed conf.Th)  
            </div>  
            """
            st.markdown(html_title, unsafe_allow_html=True)
            st.markdown(
                "Instructions: import TargetPrecisionRecallF1.")
            df = TargetPrecisionRecallF1.table
            st.table(df)

            # 计算目标级Pd和像素级Fa
            html_title = """  
            <div style="text-align:center; font-size:20px; font-weight:bold;">  
                Target-level Pd and pixel-level Fa (Fixed conf.Th) 
            </div>  
            """
            st.markdown(html_title, unsafe_allow_html=True)
            st.markdown(
                "Instructions: import TargetPdPixelFa. Note\\: Pd=Recall=TP/(TP+FN), Fa=FP/Total Pixels")
            df_pd_fa = TargetPdPixelFa.table
            st.table(df_pd_fa)


            # 显示不同欧氏距离阈值下目标级nIoU
            html_title = """  
            <div style="text-align:center; font-size:20px; font-weight:bold;">  
                Pixel-target joint level nIoU (Fixed conf.Th)   
            </div>  
            """
            st.markdown(html_title, unsafe_allow_html=True)
            st.markdown("Instructions: import HybridNormalizedIoU")
            df = HybridNormalizedIoU.table
            st.table(df)

            # 显示不同欧氏距离阈值下目标级AUC值
            html_title = """  
            <div style="text-align:center; font-size:20px; font-weight:bold;">  
                Target-level AUC (Continuous conf.Th) 
            </div>  
            """
            st.markdown(html_title, unsafe_allow_html=True)
            st.markdown("Instructions: import TargetPdPixelFaROC")
            df = TargetPdPixelFaROC.table
            st.table(df)

            # 显示目标级检测精度、召回率、F1指标
            html_title = """  
            <div style="text-align:center; font-size:20px; font-weight:bold;">  
                Target-level AP (Continuous conf.Th)  
            </div>  
            """
            st.markdown(html_title, unsafe_allow_html=True)
            st.markdown("Instructions: import TargetAveragePrecision")
            df = TargetAveragePrecision.table
            st.table(df)

            # 显示ROC曲线
            # st.line_chart(chart_data)
            figure, axes = plt.subplots()
            axes.plot(roc_data)
            axes.axis([0, 1, 0, 1])
            axes.set_title('pixel-level ROC Curve')
            axes.set_xlabel('Fa')
            axes.set_ylabel('Pd')
            st.pyplot(figure)

            # 显示PR曲线
            # st.line_chart(chart_data)
            figure, axes = plt.subplots()
            axes.plot(pr_data)
            axes.axis([0, 1, 0, 1])
            axes.set_title('pixel-level PR Curve')
            axes.set_xlabel('Recall')
            axes.set_ylabel('Precision')
            st.pyplot(figure)

        PixelPrecisionRecallF1IoU.reset()
        PixelROCPrecisionRecall.reset()
        PixelNormalizedIoU.reset()
        HybridNormalizedIoU.reset()
        TargetPrecisionRecallF1.reset()
        TargetAveragePrecision.reset()
        TargetPdPixelFa.reset()
        TargetPdPixelFaROC.reset()
        mNoCoAP.reset()

    return evaluate_info, roc_data, pr_data


def show_download_button(evaluate_info: pd.DataFrame, roc_data: pd.DataFrame, pr_data: pd.DataFrame) -> None:
    """显示下载按钮

    Args:
        evaluate_info (pd.DataFrame): 评估数据
        chart_data (pd.DataFrame): 图像数据
    """
    left_container, center_container, right_container = st.columns(3)

    with left_container:
        show_download_button_of_dataframe(
            "Download evaluate info",
            evaluate_info,
            file_name="evaluate_info.csv",
        )

    with center_container:
        show_download_button_of_dataframe(
            "Download roc_data",
            roc_data,
            file_name="roc_data.csv",
        )

    with right_container:
        show_download_button_of_dataframe(
            "Download pr_data",
            pr_data,
            file_name="pr_data.csv",
        )


def show_page() -> None:
    """显示页面"""
    st.title("Evaluate Algorithm")

    st.markdown(
        """
        Here are some description of the algorithm.
        """
    )

    is_submit = show_config_form()

    if not is_submit:
        st.stop()

    evaluate_info, roc_data, pr_data = show_evaluate_info()

    show_download_button(evaluate_info, roc_data, pr_data)


show_page()
