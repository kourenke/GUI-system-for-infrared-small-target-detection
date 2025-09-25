from typing import List
from enum import Enum
from pathlib import Path

from streamlit import markdown


__BASE_DIR__ = Path(__file__).parent.absolute()


class StyleSheet(Enum):
    BASE = "base.css"


def load_css(*selected_css: StyleSheet) -> None:
    """加载指定的 CSS 文件"""
    if len(selected_css) == 0:
        selected_css = (StyleSheet.BASE,)

    css_text: List[str] = []
    for css in selected_css:
        css_file_dir = __BASE_DIR__.joinpath(css.value)
        with open(css_file_dir, mode="r", encoding="urf-8") as __fp:
            css_text.append(__fp.read())

    markdown(css_text, unsafe_allow_html=True)
