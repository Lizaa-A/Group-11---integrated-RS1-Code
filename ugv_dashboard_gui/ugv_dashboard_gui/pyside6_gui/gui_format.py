# gui_format.py
from PySide6.QtGui import QFont

# Fonts (all monospace)
DEFAULT_FONT = QFont("Courier New", 12)
HEADER_FONT = QFont("Courier New", 16, QFont.Bold)
BIG_FONT = QFont("Courier New", 20, QFont.Bold)

# Styles and colors
class Styles:
    # Colors
    TEXT_COLOR = "#fafaf6"         # writing
    SECONDARY_TEXT_COLOR = "#252834"         # writing

    BACKGROUND_COLOR = "#252834"   # main background
    HEADER_COLOR = "#e8ffd3"       # headers / accents
    BUTTON_COLOR = "#0e2d11"       # buttons
    BUTTON_HOVER_COLOR = "#e8ffd3" # button hover
    TASK_COLOR = "#e8ffd3"         # task light green
    TASK_LIGHT_OFF = "#fafaf6"     # task inactive / gray
    PLACEHOLDER_COLOR = "#fafaf6"  # for labels, placeholders

    # Button style
    BUTTON = f"""
    QPushButton {{
        background-color: {BUTTON_COLOR};
        color: {TEXT_COLOR};
        font-family: 'Courier New';
        font-weight: bold;
        border-radius: 5px;
        padding: 10px;
        border: 1px solid {TEXT_COLOR};
    }}
    QPushButton:hover {{
        background-color: {BUTTON_HOVER_COLOR};
        color: {BUTTON_COLOR};
    }}
    """

    # Label style
    LABEL = f"font-family: 'Courier New'; font-size: 13px; color: {TEXT_COLOR};"

    # Header style
    HEADER = f"font-family: 'Courier New'; font-size: 18px; font-weight: bold; color: {HEADER_COLOR};"

    # Login input fields
    LOGIN_INPUT = f"""
    QLineEdit {{
        padding: 5px;
        border: 1px solid {TEXT_COLOR};
        border-radius: 3px;
        font-family: 'Courier New';
        color: {TEXT_COLOR};
        background-color: {BACKGROUND_COLOR};
    }}
    QPushButton {{
        padding: 5px;
        font-family: 'Courier New';
    }}
    """

    # Main window background
    WINDOW = f"background-color: {BACKGROUND_COLOR}; color: {TEXT_COLOR}; font-family: 'Courier New';"

