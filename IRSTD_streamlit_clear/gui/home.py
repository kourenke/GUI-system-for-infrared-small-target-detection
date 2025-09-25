import streamlit as st


def show_page():
    """显示页面"""

    st.markdown(
        """
        # Home

        ---

        Hi, there 👋!

        Welcome to the infrared small target detection system based on deep learning.

        This data board provides several functions:

        1. Evaluate the performance of deep learning segmentation algorithm
        2. Analyze single frame images.
        3. Analyze video you uploaded.
        4. Analyze video through infrared detectors.

        You can view the different feature pages from the left sidebar.

        Hope you enjoy!
        """
    )


st.set_page_config(
    layout="centered",
    initial_sidebar_state="expanded",
)

show_page()
