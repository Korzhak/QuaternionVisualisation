# import sys
# from view.view import QtWidgets, ViewDcmTester, pg
#
# if __name__ == "__main__":
#     timer = pg.Qt.QtCore.QTimer()
#     app = QtWidgets.QApplication(sys.argv)
#     MainWindow = QtWidgets.QMainWindow()
#     view = ViewDcmTester(MainWindow)
#     MainWindow.show()
#     sys.exit(app.exec_())


import sys
from view.new_view import QtWidgets, ViewQVisualiser, pg

if __name__ == "__main__":
    timer = pg.Qt.QtCore.QTimer()
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    view = ViewQVisualiser(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
