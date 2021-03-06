# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'layout.ui'
#
# Created by: PyQt5 UI code generator 5.15.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(581, 551)
        MainWindow.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.btn_connect = QtWidgets.QPushButton(self.centralwidget)
        self.btn_connect.setGeometry(QtCore.QRect(30, 160, 91, 51))
        self.btn_connect.setObjectName("btn_connect")
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(10, 230, 291, 20))
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.label_ip = QtWidgets.QLabel(self.centralwidget)
        self.label_ip.setGeometry(QtCore.QRect(30, 40, 55, 16))
        self.label_ip.setObjectName("label_ip")
        self.label_port = QtWidgets.QLabel(self.centralwidget)
        self.label_port.setGeometry(QtCore.QRect(30, 90, 55, 16))
        self.label_port.setObjectName("label_port")
        self.input_ip = QtWidgets.QLineEdit(self.centralwidget)
        self.input_ip.setGeometry(QtCore.QRect(120, 40, 113, 22))
        self.input_ip.setObjectName("input_ip")
        self.input_port = QtWidgets.QLineEdit(self.centralwidget)
        self.input_port.setGeometry(QtCore.QRect(120, 90, 113, 22))
        self.input_port.setObjectName("input_port")
        self.btn_file_explorer = QtWidgets.QPushButton(self.centralwidget)
        self.btn_file_explorer.setGeometry(QtCore.QRect(260, 260, 31, 21))
        self.btn_file_explorer.setObjectName("btn_file_explorer")
        self.line_2 = QtWidgets.QFrame(self.centralwidget)
        self.line_2.setGeometry(QtCore.QRect(300, 10, 20, 511))
        self.line_2.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.label_current_wobj = QtWidgets.QLabel(self.centralwidget)
        self.label_current_wobj.setGeometry(QtCore.QRect(320, 10, 51, 20))
        self.label_current_wobj.setObjectName("label_current_wobj")
        self.btn_send_gCode = QtWidgets.QPushButton(self.centralwidget)
        self.btn_send_gCode.setGeometry(QtCore.QRect(190, 160, 91, 51))
        self.btn_send_gCode.setObjectName("btn_send_gCode")
        self.label_connection_status = QtWidgets.QLabel(self.centralwidget)
        self.label_connection_status.setGeometry(QtCore.QRect(320, 70, 91, 20))
        self.label_connection_status.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_connection_status.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_connection_status.setObjectName("label_connection_status")
        self.g_code_viewer = QtWidgets.QTextBrowser(self.centralwidget)
        self.g_code_viewer.setGeometry(QtCore.QRect(25, 291, 261, 231))
        self.g_code_viewer.setObjectName("g_code_viewer")
        self.line_3 = QtWidgets.QFrame(self.centralwidget)
        self.line_3.setGeometry(QtCore.QRect(320, 40, 241, 20))
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.label_tool_status = QtWidgets.QLabel(self.centralwidget)
        self.label_tool_status.setGeometry(QtCore.QRect(320, 100, 91, 20))
        self.label_tool_status.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_tool_status.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_tool_status.setObjectName("label_tool_status")
        self.indicator_connection = QtWidgets.QLabel(self.centralwidget)
        self.indicator_connection.setGeometry(QtCore.QRect(510, 70, 16, 16))
        self.indicator_connection.setAutoFillBackground(False)
        self.indicator_connection.setStyleSheet("background-color: rgb(255, 0, 0);")
        self.indicator_connection.setFrameShape(QtWidgets.QFrame.Box)
        self.indicator_connection.setText("")
        self.indicator_connection.setObjectName("indicator_connection")
        self.indicator_tool = QtWidgets.QLabel(self.centralwidget)
        self.indicator_tool.setGeometry(QtCore.QRect(510, 100, 16, 16))
        self.indicator_tool.setAutoFillBackground(False)
        self.indicator_tool.setStyleSheet("background-color: rgb(255, 0, 0);")
        self.indicator_tool.setFrameShape(QtWidgets.QFrame.Box)
        self.indicator_tool.setText("")
        self.indicator_tool.setObjectName("indicator_tool")
        self.btn_set_tool_wobj = QtWidgets.QPushButton(self.centralwidget)
        self.btn_set_tool_wobj.setGeometry(QtCore.QRect(390, 460, 91, 51))
        self.btn_set_tool_wobj.setObjectName("btn_set_tool_wobj")
        self.label_ip_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_ip_2.setGeometry(QtCore.QRect(320, 150, 55, 16))
        self.label_ip_2.setObjectName("label_ip_2")
        self.label_ip_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_ip_3.setGeometry(QtCore.QRect(320, 270, 71, 16))
        self.label_ip_3.setObjectName("label_ip_3")
        self.label_ip_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_ip_4.setGeometry(QtCore.QRect(360, 180, 55, 16))
        self.label_ip_4.setObjectName("label_ip_4")
        self.label_ip_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_ip_5.setGeometry(QtCore.QRect(360, 210, 55, 16))
        self.label_ip_5.setObjectName("label_ip_5")
        self.label_ip_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_ip_6.setGeometry(QtCore.QRect(360, 240, 55, 16))
        self.label_ip_6.setObjectName("label_ip_6")
        self.label_ip_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_ip_7.setGeometry(QtCore.QRect(360, 300, 55, 16))
        self.label_ip_7.setObjectName("label_ip_7")
        self.label_ip_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_ip_8.setGeometry(QtCore.QRect(360, 330, 55, 16))
        self.label_ip_8.setObjectName("label_ip_8")
        self.label_ip_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_ip_9.setGeometry(QtCore.QRect(360, 360, 55, 16))
        self.label_ip_9.setObjectName("label_ip_9")
        self.label_ip_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_ip_10.setGeometry(QtCore.QRect(360, 390, 55, 16))
        self.label_ip_10.setObjectName("label_ip_10")
        self.label_x_pos = QtWidgets.QLabel(self.centralwidget)
        self.label_x_pos.setGeometry(QtCore.QRect(420, 180, 131, 20))
        self.label_x_pos.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border: 1px solid grey;")
        self.label_x_pos.setText("")
        self.label_x_pos.setObjectName("label_x_pos")
        self.label_y_pos = QtWidgets.QLabel(self.centralwidget)
        self.label_y_pos.setGeometry(QtCore.QRect(420, 210, 131, 20))
        self.label_y_pos.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border: 1px solid grey;")
        self.label_y_pos.setText("")
        self.label_y_pos.setObjectName("label_y_pos")
        self.label_z_pos = QtWidgets.QLabel(self.centralwidget)
        self.label_z_pos.setGeometry(QtCore.QRect(420, 240, 131, 20))
        self.label_z_pos.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border: 1px solid grey;")
        self.label_z_pos.setText("")
        self.label_z_pos.setObjectName("label_z_pos")
        self.label_q1 = QtWidgets.QLabel(self.centralwidget)
        self.label_q1.setGeometry(QtCore.QRect(420, 300, 131, 20))
        self.label_q1.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border: 1px solid grey;")
        self.label_q1.setText("")
        self.label_q1.setObjectName("label_q1")
        self.label_q2 = QtWidgets.QLabel(self.centralwidget)
        self.label_q2.setGeometry(QtCore.QRect(420, 330, 131, 20))
        self.label_q2.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border: 1px solid grey;")
        self.label_q2.setText("")
        self.label_q2.setObjectName("label_q2")
        self.label_q3 = QtWidgets.QLabel(self.centralwidget)
        self.label_q3.setGeometry(QtCore.QRect(420, 360, 131, 20))
        self.label_q3.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border: 1px solid grey;")
        self.label_q3.setText("")
        self.label_q3.setObjectName("label_q3")
        self.label_q4 = QtWidgets.QLabel(self.centralwidget)
        self.label_q4.setGeometry(QtCore.QRect(420, 390, 131, 20))
        self.label_q4.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border: 1px solid grey;")
        self.label_q4.setText("")
        self.label_q4.setObjectName("label_q4")
        self.line_6 = QtWidgets.QFrame(self.centralwidget)
        self.line_6.setGeometry(QtCore.QRect(320, 130, 241, 20))
        self.line_6.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_6.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_6.setObjectName("line_6")
        self.label_file_path = QtWidgets.QLabel(self.centralwidget)
        self.label_file_path.setGeometry(QtCore.QRect(20, 260, 231, 22))
        self.label_file_path.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border: 1px solid grey;")
        self.label_file_path.setText("")
        self.label_file_path.setObjectName("label_file_path")
        self.line_7 = QtWidgets.QFrame(self.centralwidget)
        self.line_7.setGeometry(QtCore.QRect(320, 420, 241, 20))
        self.line_7.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_7.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_7.setObjectName("line_7")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 581, 21))
        self.menubar.setObjectName("menubar")
        self.menuZone = QtWidgets.QMenu(self.menubar)
        self.menuZone.setObjectName("menuZone")
        self.menuSpeed = QtWidgets.QMenu(self.menubar)
        self.menuSpeed.setObjectName("menuSpeed")
        MainWindow.setMenuBar(self.menubar)
        self.actionv5 = QtWidgets.QAction(MainWindow)
        self.actionv5.setCheckable(True)
        self.actionv5.setObjectName("actionv5")
        self.actionv10 = QtWidgets.QAction(MainWindow)
        self.actionv10.setCheckable(True)
        self.actionv10.setObjectName("actionv10")
        self.actionv20 = QtWidgets.QAction(MainWindow)
        self.actionv20.setObjectName("actionv20")
        self.actionv30 = QtWidgets.QAction(MainWindow)
        self.actionv30.setObjectName("actionv30")
        self.actionv40 = QtWidgets.QAction(MainWindow)
        self.actionv40.setObjectName("actionv40")
        self.actionv50 = QtWidgets.QAction(MainWindow)
        self.actionv50.setCheckable(True)
        self.actionv50.setObjectName("actionv50")
        self.actionv60 = QtWidgets.QAction(MainWindow)
        self.actionv60.setObjectName("actionv60")
        self.actionv80 = QtWidgets.QAction(MainWindow)
        self.actionv80.setObjectName("actionv80")
        self.actionv100 = QtWidgets.QAction(MainWindow)
        self.actionv100.setCheckable(True)
        self.actionv100.setObjectName("actionv100")
        self.actionv150 = QtWidgets.QAction(MainWindow)
        self.actionv150.setObjectName("actionv150")
        self.actionv200 = QtWidgets.QAction(MainWindow)
        self.actionv200.setCheckable(True)
        self.actionv200.setObjectName("actionv200")
        self.actionv300 = QtWidgets.QAction(MainWindow)
        self.actionv300.setCheckable(True)
        self.actionv300.setObjectName("actionv300")
        self.actionv400 = QtWidgets.QAction(MainWindow)
        self.actionv400.setCheckable(True)
        self.actionv400.setObjectName("actionv400")
        self.actionv500 = QtWidgets.QAction(MainWindow)
        self.actionv500.setCheckable(True)
        self.actionv500.setObjectName("actionv500")
        self.actionv600 = QtWidgets.QAction(MainWindow)
        self.actionv600.setObjectName("actionv600")
        self.actionv800 = QtWidgets.QAction(MainWindow)
        self.actionv800.setObjectName("actionv800")
        self.actionv1000 = QtWidgets.QAction(MainWindow)
        self.actionv1000.setCheckable(True)
        self.actionv1000.setObjectName("actionv1000")
        self.actionfine = QtWidgets.QAction(MainWindow)
        self.actionfine.setCheckable(True)
        self.actionfine.setObjectName("actionfine")
        self.actionz0 = QtWidgets.QAction(MainWindow)
        self.actionz0.setCheckable(True)
        self.actionz0.setObjectName("actionz0")
        self.actionz1 = QtWidgets.QAction(MainWindow)
        self.actionz1.setCheckable(True)
        self.actionz1.setObjectName("actionz1")
        self.actionz5 = QtWidgets.QAction(MainWindow)
        self.actionz5.setCheckable(True)
        self.actionz5.setObjectName("actionz5")
        self.actioncurrent_zone = QtWidgets.QAction(MainWindow)
        self.actioncurrent_zone.setObjectName("actioncurrent_zone")
        self.actionz10 = QtWidgets.QAction(MainWindow)
        self.actionz10.setCheckable(True)
        self.actionz10.setObjectName("actionz10")
        self.actionz15 = QtWidgets.QAction(MainWindow)
        self.actionz15.setObjectName("actionz15")
        self.actionz20 = QtWidgets.QAction(MainWindow)
        self.actionz20.setObjectName("actionz20")
        self.actionz30 = QtWidgets.QAction(MainWindow)
        self.actionz30.setObjectName("actionz30")
        self.actionz40 = QtWidgets.QAction(MainWindow)
        self.actionz40.setObjectName("actionz40")
        self.actionz50 = QtWidgets.QAction(MainWindow)
        self.actionz50.setCheckable(True)
        self.actionz50.setObjectName("actionz50")
        self.actionz60 = QtWidgets.QAction(MainWindow)
        self.actionz60.setObjectName("actionz60")
        self.actionz80 = QtWidgets.QAction(MainWindow)
        self.actionz80.setObjectName("actionz80")
        self.actionz200 = QtWidgets.QAction(MainWindow)
        self.actionz200.setCheckable(True)
        self.actionz200.setObjectName("actionz200")
        self.actionz100 = QtWidgets.QAction(MainWindow)
        self.actionz100.setCheckable(True)
        self.actionz100.setObjectName("actionz100")
        self.actionz150 = QtWidgets.QAction(MainWindow)
        self.actionz150.setObjectName("actionz150")
        self.actioncurrent_Tool = QtWidgets.QAction(MainWindow)
        self.actioncurrent_Tool.setObjectName("actioncurrent_Tool")
        self.actiontool0 = QtWidgets.QAction(MainWindow)
        self.actiontool0.setObjectName("actiontool0")
        self.actioncurrent_wobj = QtWidgets.QAction(MainWindow)
        self.actioncurrent_wobj.setObjectName("actioncurrent_wobj")
        self.actionwobj0 = QtWidgets.QAction(MainWindow)
        self.actionwobj0.setObjectName("actionwobj0")
        self.actioncurrent_speed = QtWidgets.QAction(MainWindow)
        self.actioncurrent_speed.setObjectName("actioncurrent_speed")
        self.actionbufferspeed = QtWidgets.QAction(MainWindow)
        self.actionbufferspeed.setObjectName("actionbufferspeed")
        self.menuZone.addAction(self.actionfine)
        self.menuZone.addAction(self.actionz0)
        self.menuZone.addAction(self.actionz1)
        self.menuZone.addAction(self.actionz5)
        self.menuZone.addAction(self.actionz10)
        self.menuZone.addAction(self.actionz50)
        self.menuZone.addAction(self.actionz100)
        self.menuZone.addAction(self.actionz200)
        self.menuSpeed.addAction(self.actionv5)
        self.menuSpeed.addAction(self.actionv10)
        self.menuSpeed.addAction(self.actionv50)
        self.menuSpeed.addAction(self.actionv100)
        self.menuSpeed.addAction(self.actionv200)
        self.menuSpeed.addAction(self.actionv300)
        self.menuSpeed.addAction(self.actionv400)
        self.menuSpeed.addAction(self.actionv500)
        self.menuSpeed.addAction(self.actionv1000)
        self.menubar.addAction(self.menuSpeed.menuAction())
        self.menubar.addAction(self.menuZone.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.btn_connect.setText(_translate("MainWindow", "Connect \n"
"to Robot"))
        self.label_ip.setText(_translate("MainWindow", "IP"))
        self.label_port.setText(_translate("MainWindow", "PORT"))
        self.btn_file_explorer.setText(_translate("MainWindow", "..."))
        self.label_current_wobj.setText(_translate("MainWindow", "Status:"))
        self.btn_send_gCode.setText(_translate("MainWindow", "Send \n"
"to Robot"))
        self.label_connection_status.setText(_translate("MainWindow", "Connected:"))
        self.label_tool_status.setText(_translate("MainWindow", "Tool on:"))
        self.btn_set_tool_wobj.setText(_translate("MainWindow", "Set \n"
"tool/wobj"))
        self.label_ip_2.setText(_translate("MainWindow", "Position:"))
        self.label_ip_3.setText(_translate("MainWindow", "Orientation:"))
        self.label_ip_4.setText(_translate("MainWindow", "x axis:"))
        self.label_ip_5.setText(_translate("MainWindow", "y axis:"))
        self.label_ip_6.setText(_translate("MainWindow", "z axis:"))
        self.label_ip_7.setText(_translate("MainWindow", "q1:"))
        self.label_ip_8.setText(_translate("MainWindow", "q2:"))
        self.label_ip_9.setText(_translate("MainWindow", "q3:"))
        self.label_ip_10.setText(_translate("MainWindow", "q4:"))
        self.menuZone.setTitle(_translate("MainWindow", "Zone"))
        self.menuSpeed.setTitle(_translate("MainWindow", "Speed"))
        self.actionv5.setText(_translate("MainWindow", "v5"))
        self.actionv10.setText(_translate("MainWindow", "v10"))
        self.actionv20.setText(_translate("MainWindow", "v20"))
        self.actionv30.setText(_translate("MainWindow", "v30"))
        self.actionv40.setText(_translate("MainWindow", "v40"))
        self.actionv50.setText(_translate("MainWindow", "v50"))
        self.actionv60.setText(_translate("MainWindow", "v60"))
        self.actionv80.setText(_translate("MainWindow", "v80"))
        self.actionv100.setText(_translate("MainWindow", "v100"))
        self.actionv150.setText(_translate("MainWindow", "v150"))
        self.actionv200.setText(_translate("MainWindow", "v200"))
        self.actionv300.setText(_translate("MainWindow", "v300"))
        self.actionv400.setText(_translate("MainWindow", "v400"))
        self.actionv500.setText(_translate("MainWindow", "v500"))
        self.actionv600.setText(_translate("MainWindow", "v600"))
        self.actionv800.setText(_translate("MainWindow", "v800"))
        self.actionv1000.setText(_translate("MainWindow", "v1000"))
        self.actionfine.setText(_translate("MainWindow", "fine"))
        self.actionz0.setText(_translate("MainWindow", "z0"))
        self.actionz1.setText(_translate("MainWindow", "z1"))
        self.actionz5.setText(_translate("MainWindow", "z5"))
        self.actioncurrent_zone.setText(_translate("MainWindow", "current zone"))
        self.actionz10.setText(_translate("MainWindow", "z10"))
        self.actionz15.setText(_translate("MainWindow", "z15"))
        self.actionz20.setText(_translate("MainWindow", "z20"))
        self.actionz30.setText(_translate("MainWindow", "z30"))
        self.actionz40.setText(_translate("MainWindow", "z40"))
        self.actionz50.setText(_translate("MainWindow", "z50"))
        self.actionz60.setText(_translate("MainWindow", "z60"))
        self.actionz80.setText(_translate("MainWindow", "z80"))
        self.actionz200.setText(_translate("MainWindow", "z200"))
        self.actionz100.setText(_translate("MainWindow", "z100"))
        self.actionz150.setText(_translate("MainWindow", "z150"))
        self.actioncurrent_Tool.setText(_translate("MainWindow", "current Tool"))
        self.actiontool0.setText(_translate("MainWindow", "tool0"))
        self.actioncurrent_wobj.setText(_translate("MainWindow", "current wobj"))
        self.actionwobj0.setText(_translate("MainWindow", "wobj0"))
        self.actioncurrent_speed.setText(_translate("MainWindow", "current speed"))
        self.actionbufferspeed.setText(_translate("MainWindow", "bufferspeed"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
