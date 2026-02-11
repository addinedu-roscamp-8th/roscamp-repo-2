import sys
from PyQt6.QtWidgets import QApplication

from gui_pkg.main_window import RobotControlSystem, initialize_database
from gui_pkg.gui_node import GuiNode


def main():
    # 1. DB 초기화
    initialize_database()

    # 2. Qt Application 생성
    app = QApplication(sys.argv)

    # 3. ROS GUI 노드 생성 (QThread 기반)
    ros_thread = GuiNode()

    # 4. 메인 윈도우 생성
    window = RobotControlSystem(ros_thread)
    window.show()

    # 5. Qt 실행
    exit_code = app.exec()

    # 6. 종료 처리
    ros_thread.stop()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()