import sys
from PyQt5.QtWidgets import QApplication
from framework_window import PMProgrammingFramework

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PMProgrammingFramework()
    window.show()
    sys.exit(app.exec_())
