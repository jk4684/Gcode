import socket
import sys
import threading
import queue
from math import sqrt, sin, cos, atan2, asin, pi
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QWidget
from layout import Ui_MainWindow
from toolWobjConfigurator import Ui_toolWobjConfigurator
from time import sleep

BUFFER = 4096
HEADERSIZE = 10
HOME = ['750.0 ', '0.0 ', '1185.0 ',
        '0.00000 ', '0.00000 ', '1.00000 ', '0.00000',
        '100']

# ================#
# gCode commands #
# ================#
GCOMM = ["G00",  # rapid positioning
         "G01",  # linear move
         "G02",  # circular clockwise
         "G03",  # circular cntrclkwise
         "G17",  # XY plane
         "G18",  # XZ plane
         "G19",  # YZ plane
         "G20",  # inches
         "G21",  # milimeters
         "G28",  # return home
         "G90",  # absolute mode
         "G91",  # relative mode
         "M00",  # program stop
         "M2",  # end of program
         "M30",  # end of program
         "M3",  # spindle on
         "M5"]  # spindle stop


class Window(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(Window, self).__init__()
        self.setupUi(self)
        self.configurator = Configurator(self)
        self.connected = False
        self.path = None
        self.instructionCompleted = False

        self.receiveThread = threading.Thread(target=self.receive, daemon=True)
        self.sendThread = threading.Thread(target=self.send, daemon=True)
        self.send_que = queue.Queue()

        self.client_socket = socket

        self.orientation = f'{HOME[3]}{HOME[4]}{HOME[5]}{HOME[6]}'  # quaternion equivalent of eye RotMatrix
        self.conversion = 1  # conversion mm/inch
        self.tool_on = False

        self.currentPosition = [0, 0, 0]  # current position of TCP get from server
        self.previousGCodePosition = [0, 0, 0]  # end point of previous gCode move

        self.gCode = []
        self.x, self.y, self.z, self.feedrate = HOME[0], HOME[1], HOME[2], HOME[-1]

        # =====================buttons=========================== #

        self.btn_connect.pressed.connect(lambda: self.connect_to_robot('localhost', '5000'))
        self.btn_send_gCode.pressed.connect(lambda: self.gCode_handler())
        self.btn_file_explorer.pressed.connect(lambda: self.open_file())
        self.btn_send.pressed.connect(lambda: self.put_send(self.input.text()))
        # self.btn_send.pressed.connect(lambda: self.gCode_to_rapid(self.input.text()))
        self.btn_set_tool_wobj.pressed.connect(lambda: self.open_configurator())

    # ===================gCode_to_rapid====================== #

    def open_file(self):
        fname = QFileDialog.getOpenFileName(self,
                                            'Open file',
                                            'c:\\Users\jakak\Desktop\Jaka\RobotStudio\IndustrijskiSeminar\Gcode\GcodeFiles',
                                            'G-code files (*.ngc)')
        if len(fname[0]) != 0:
            file = open(fname[0])
            self.g_code_viewer.setText(file.read())
            self.label_file_path.setText(fname[0])
            # self.gCode_handler(fname[0])
            self.path = fname[0]

    def gCode_handler(self):  # when "send to robot" btn is pressed
        self.gCode = []
        try:
            file = open(self.path).read().split('\n')
            file = [line for line in file if line]  # gets rid of empty strings "" in gCode file
            for line in file:
                if line[0] == 'G' or line[0] == 'M':
                    instr = line.split(' ')
                    self.gCode.append(instr)
            self.gCode_to_rapid(self.gCode[0])
        except:
            pass

    def gCode_to_rapid(self, gCode_line):
        if gCode_line[0] == "G00":
            msg = self.move_handler(gCode_line).split(':')
            msg = f'02 {msg[0]} {msg[1]}'
            self.put_send(msg)

        elif gCode_line[0] == "G01":
            msg = self.move_handler(gCode_line).split(':')
            msg = f'01 {msg[0]} {msg[1]}'
            print(msg)
            self.put_send(msg)

        elif gCode_line[0] == "G02":
            msg = self.move_handler(gCode_line).split(':')
            msg1 = f'35 {msg[0]} {msg[2]}'
            msg2 = f'36 {msg[1]} {msg[2]}'
            print(msg1, msg2)
            self.put_send(msg1)
            sleep(0.01)
            self.put_send(msg2)

        elif gCode_line[0] == "G03":
            msg = self.move_handler(gCode_line).split(':')
            msg1 = f'35 {msg[0]} {msg[2]}'
            msg2 = f'36 {msg[1]} {msg[2]}'
            self.put_send(msg1)
            sleep(0.01)
            self.put_send(msg2)

        elif gCode_line[0] == "G17":
            self.orientation = f'0.00000 0.00000 1.00000 0.00000'  # -normala v xy ravnino
            self.instructionCompleted = True
            self.receive_handler(None)

        elif gCode_line[0] == "G18":
            self.orientation = f'{str(sqrt(0.5))} {str(sqrt(0.5))} 0.00000 0.00000'  # -normala v xz ravnino
            self.instructionCompleted = True
            self.receive_handler(None)

        elif gCode_line[0] == "G19":
            self.orientation = f'{str(sqrt(0.5))} 0.00000 {str(sqrt(0.5))} 0.00000'  # -normala v yz ravnino
            self.instructionCompleted = True
            self.receive_handler(None)

        elif gCode_line[0] == "G20":
            self.conversion = 25.4

        elif gCode_line[0] == "G21":
            self.conversion = 1
            self.instructionCompleted = True
            self.receive_handler(None)

        elif gCode_line[0] == "G28":
            msg = f'02 '
            for i in range(len(HOME) - 1):
                msg += HOME[i]

        elif gCode_line[0] == "G90":
            self.instructionCompleted = True
            self.receive_handler(None)

        # elif gCode_line[0] == "G91":
        #    pass

        # elif gCode_line[0] == "M00":    # program stop
        #    pass

        elif gCode_line[0] == "M02":  # end of program
            self.tool_on = False
            msg = f'02 '
            for i in range(len(HOME) - 1):
                msg += HOME[i]
            self.put_send(msg)

        elif gCode_line[0] == "M03":
            self.tool_on = True
            self.indicator_tool.setStyleSheet("background-color: green")
            self.instructionCompleted = True
            self.receive_handler(None)

        elif gCode_line[0] == "M05":
            self.tool_on = False
            self.indicator_tool.setStyleSheet("background-color: red")
            self.instructionCompleted = True
            self.receive_handler(None)
        pass

    def move_handler(self, gCode_line):
        self.x = gCode_line[1][1::]  # gets x coordinates
        self.y = gCode_line[2][1::]  # gets y coordinates
        self.z = gCode_line[3][1::]  # gets z coordinates
        str_endPoint = self.x + ' ' + self.y + ' ' + self.z
        str_midPoint = ''

        if gCode_line[0] == 'G01':
            if len(gCode_line) == 5:
                self.feedrate = gCode_line[4][1::]  # if feedrate then change it else not
                self.change_speed()

        elif gCode_line[0] == 'G02' or gCode_line[0] == 'G03':
            if len(gCode_line) == 7:  # če ima g code na koncu presledek je to problem
                self.feedrate = gCode_line[6][1::]
                self.change_speed()
            offset = [float(gCode_line[4][1::]), float(gCode_line[5][1::]), 0.0]  # z koordinata nima offseta
            endPoint = [float(self.x), float(self.y), float(self.z)]
            direction = 'clockwise'
            if gCode_line[0] == 'G03':
                direction = '-clockwise'
            str_midPoint = self.moveC(offset, endPoint, direction)

        if len(str_midPoint) > 0:
            msg = str_midPoint + str_endPoint  # + ' '  + self.feedrate + ' '
        else:
            msg = str_endPoint  # + ' '  + self.feedrate + ' '
        self.previousGCodePosition = [float(self.x), float(self.y), float(self.z)]  # saves endpoints
        return f'{msg}:{self.orientation}'

    def moveC(self, centerCoord, finCoord, direction):
        x_0, y_0, z_0 = self.previousGCodePosition
        x_c, y_c, z_c = centerCoord[0] + x_0, centerCoord[1] + y_0, centerCoord[2] + z_0
        x_1, y_1, z_1 = finCoord
        r = sqrt((x_c - x_0) ** 2 + (y_c - y_0) ** 2)
        k = sqrt((x_0 - x_1) ** 2 + (y_0 - y_1) ** 2) / 2
        a = asin(k / r)
        fi_1 = atan2((y_1 - y_c), (x_1 - x_c))
        fi_2 = atan2((y_0 - y_c), (x_0 - x_c))
        if direction == 'clockwise':
            midCoord = [str(x_c + cos(fi_1 + a) * r), str(y_c + sin(fi_1 + a) * r), str(z_0)]
        elif direction == '-clockwise':
            midCoord = [str(x_c + cos(fi_2 + a) * r), str(y_c + sin(fi_2 + a) * r), str(z_0)]
        else:
            print('Incorrect direction argument')
            midCoord = None
        return f'{midCoord[0]} {midCoord[1]} {midCoord[2]}:'

    def change_speed(self):
        msg = f'08 {self.feedrate:.1f} 50.00 0.0 0.00'
        self.put_send(msg)

    def get_position(self):
        pass

    def create_message(self):
        pass

    # ===============connect====================== #

    def connect_to_robot(self, ip, port):
        if len(ip) == 0 or len(port) == 0:
            print('Please input desired ip and port')
        else:
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((ip, int(port)))
                self.client_socket.setblocking(False)
                self.connected = True
                if not self.receiveThread.isAlive():
                    self.receiveThread.start()
                    self.sendThread.start()
            except socket.error:
                print("didn't connect:(")

    # ===============receive====================== #

    def receive(self):
        while True:
            try:
                data = self.client_socket.recv(BUFFER)
                data = data.decode('utf-8')
                if data is not None:
                    if data.find('#') != -1:
                        self.receive_handler(data)
                        data = None
            except:
                continue

    def receive_handler(self, data):
        # print('data: ', data)
        if data == 'C#':
            self.connected = True
            self.indicator_connection.setStyleSheet("background-color: green")
        elif data == 'LC#':
            self.connected = False
            self.indicator_connection.setStyleSheet("background-color: red")
            self.client_socket.shutdown(socket.SHUT_RDWR)
            self.client_socket.close()
        elif data == 'MNC#':
            print('move not completed')
        elif data == 'IC#' or self.instructionCompleted:
            self.instructionCompleted = False
            self.gCode.pop(0)
            self.gCode_to_rapid(self.gCode[0])
        elif data[0] == '!':
            coord = data.split(' ')
            self.label_x_pos.setText(coord[1])
            self.label_y_pos.setText(coord[2])
            self.label_z_pos.setText(coord[3])
            self.label_q1.setText(coord[4])
            self.label_q2.setText(coord[5])
            self.label_q3.setText(coord[6])
            self.label_q4.setText(coord[7])
        else:
            pass

    # =================send======================= #

    def send(self):
        while True:
            data = self.send_que.get()
            while data is not None:
                try:
                    if self.connected:
                        self.client_socket.send((data + ' #').encode('utf-8'))
                        data = None
                except socket.error:
                    print('data not sent')

    def put_send(self, data):
        self.send_que.put(data)

    def open_configurator(self):
        self.configurator.show()


class Configurator(QWidget, Ui_toolWobjConfigurator):
    def __init__(self, mainwindow):
        super(Configurator, self).__init__()
        self.setupUi(self)
        self.x = mainwindow

        self.btn_apply.pressed.connect(lambda: self.apply_values())
        self.btn_cancel.pressed.connect(lambda: self.close())
        self.btn_ok.pressed.connect(lambda: self.confirm())

    def apply_values(self):
        if self.checkBox_wobj.isChecked():
            wobj = [str(self.input_wobj_x.value()),
                    str(self.input_wobj_y.value()),
                    str(self.input_wobj_z.value()),
                    str(self.input_wobj_q1.value()),
                    str(self.input_wobj_q2.value()),
                    str(self.input_wobj_q3.value()),
                    str(self.input_wobj_q4.value())]
            wobj_msg = f'07 {wobj[0]} {wobj[1]} {wobj[2]} {wobj[3]} {wobj[4]} {wobj[5]} {wobj[6]}'
            self.x.put_send(wobj_msg)

        if self.checkBox_tool.isChecked():
            tool = [str(self.input_tool_x.value()),
                    str(self.input_tool_y.value()),
                    str(self.input_tool_z.value()),
                    str(self.input_tool_q1.value()),
                    str(self.input_tool_q2.value()),
                    str(self.input_tool_q3.value()),
                    str(self.input_tool_q4.value())]
            tool_msg = f'06 {tool[0]} {tool[1]} {tool[2]} {tool[3]} {tool[4]} {tool[5]} {tool[6]}'
            self.x.put_send(tool_msg)

    def confirm(self):
        self.apply_values()
        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    wnd = Window()
    wnd.show()
    sys.exit(app.exec_())
