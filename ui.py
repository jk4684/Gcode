import socket
import sys
import threading
import queue
from math import sqrt, sin, cos, atan2, asin, pi
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QWidget, QDialog, QLabel
from layout import Ui_MainWindow
from toolWobjConfigurator import Ui_toolWobjConfigurator
from time import sleep
from timeit import default_timer as timer

BUFFER = 4096
HEADERSIZE = 10
HOME = ['750.0 ', '0.0 ', '1185.0 ',
        '0.00000 ', '0.00000 ', '1.00000 ', '0.00000',
        '100']
HOME_OFFSET = ['0', '0', '0', '0', '0', '0', '0']
TOOL_INIT = ['0', '0', '0', '1', '0', '0', '0']
WOBJ_INIT = ['0', '0', '0', '1', '0', '0', '0']
# ================#
# gCode commands #
# ================#
GCOMM = ["G00",     # rapid positioning
         "G01",     # linear move
         "G02",     # circular clockwise
         "G03",     # circular cntrclkwise
         "G17",     # XY plane
         "G18",     # XZ plane
         "G19",     # YZ plane
         "G20",     # inches
         "G21",     # milimeters
         "G28",     # return home
         "G90",     # absolute mode
         "G91",     # relative mode
         "M00",     # program stop
         "M2",      # end of program
         "M30",     # end of program
         "M3",      # spindle on
         "M5"]      # spindle stop


class Window(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(Window, self).__init__()
        self.setupUi(self)
        self.configurator = Configurator(self)
        self.popup = QDialog(self)
        self.connected = False
        self.path = None
        self.instructionCompleted = False

        self.receiveThread = threading.Thread(target=self.receive, daemon=True)
        self.sendThread = threading.Thread(target=self.send, daemon=True)
        self.send_que = queue.Queue()

        self.client_socket = socket

        self.orientation = f'{HOME[3]}{HOME[4]}{HOME[5]}{HOME[6]}'      # quaternion equivalent of eye RotMatrix
        self.plane = 1                                                  # 1 = xy; 2 = xz; 3 = yz
        self.conversion = 1                                             # conversion mm/inch
        self.tool_on = False

        self.currentPosition = [0, 0, 0]                                # current position of TCP get from server
        self.previousGCodePosition = [0, 0, 0]                          # end point of previous gCode move

        self.gCode = []
        self.x, self.y, self.z, self.feedrate = HOME[0], HOME[1], HOME[2], HOME[-1]

    # =====================buttons=========================== #

        if not self.connected:
            self.btn_connect.released.connect(lambda: self.connect_to_robot('localhost', '5000'))
        elif self.connected:
            self.btn_connect.released.connect(lambda: self.put_send('99'))
        # self.btn_send.setEnabled(False)
        self.btn_send_gCode.setEnabled(False)
        self.btn_set_tool_wobj.setEnabled(False)
        self.btn_file_explorer.setEnabled(False)
        self.btn_send_gCode.clicked.connect(lambda: self.gCode_handler())
        self.btn_file_explorer.clicked.connect(lambda: self.open_file())
        # self.btn_send.pressed.connect(self.show_popup)
        # self.btn_send.pressed.connect(lambda: self.put_send(self.input.text()))
        # self.btn_send.clicked.connect(lambda: self.gCode_to_rapid(self.input.text()))
        self.btn_set_tool_wobj.clicked.connect(lambda: self.open_configurator())
        self.actionv10.triggered.connect(lambda: self.change_speed('10'))
        self.actionv100.triggered.connect(lambda: self.change_speed('100'))
        self.actionv1000.triggered.connect(lambda: self.change_speed('1000'))
        self.actionv200.triggered.connect(lambda: self.change_speed('200'))
        self.actionv300.triggered.connect(lambda: self.change_speed('300'))
        self.actionv400.triggered.connect(lambda: self.change_speed('400'))
        self.actionv500.triggered.connect(lambda: self.change_speed('500'))
        self.actionv5.triggered.connect(lambda: self.change_speed('5'))
        self.actionv50.triggered.connect(lambda: self.change_speed('50'))
        self.actionfine.triggered.connect(lambda: self.change_zone('fine'))
        self.actionz0.triggered.connect(lambda: self.change_zone('z0'))
        self.actionz1.triggered.connect(lambda: self.change_zone('z1'))
        self.actionz5.triggered.connect(lambda: self.change_zone('z5'))
        self.actionz10.triggered.connect(lambda: self.change_zone('z10'))
        self.actionz50.triggered.connect(lambda: self.change_zone('z50'))
        self.actionz100.triggered.connect(lambda: self.change_zone('z100'))
        self.actionz200.triggered.connect(lambda: self.change_zone('z200'))


    # ===================gCode_to_rapid====================== #

    def open_file(self):
        fname = QFileDialog.getOpenFileName(self,
                                            'Open file',
                                            'c:\\Users\jakak\Desktop\Jaka\RobotStudio\IndustrijskiSeminar\Gcode\GcodeFiles')
        if len(fname[0]) != 0:
            file = open(fname[0])
            self.g_code_viewer.setText(file.read())
            self.label_file_path.setText(fname[0])
            self.path = fname[0]

    def gCode_handler(self):                # when "send to robot" btn is pressed
        self.gCode = []
        instr = ''
        try:
            file = open(self.path).read().split('\n')
            file = [line for line in file if line]  # gets rid of empty strings "" in gCode file
            for line in file:
                if line[0] == 'G' or line[0] == 'M':
                    instr = line.split(' ')
                    self.gCode.append(instr)
            self.gCode_to_rapid(self.gCode)
        except:
            pass

    def gCode_to_rapid(self, gCode):
        global HOME_OFFSET
        msg = '31'
        self.put_send(msg)          # clear buffer
        msg = '03'
        self.put_send(msg)          # get cartesian coordinates of a robot
        for gCode_line in gCode:
            sleep(0.3)
            if gCode_line[0] == "G00":
                msg = self.move_handler(gCode_line).split(':')
                msg = f'30 {msg[0]} {msg[1]} {self.feedrate} 2'         #JmoveEndPoint
                self.put_send(msg)
    
            elif gCode_line[0] == "G01":
                msg = self.move_handler(gCode_line).split(':')
                msg = f'30 {msg[0]} {msg[1]} {self.feedrate} 1'         # linearEndPoint
                print(msg)
                self.put_send(msg)
    
            elif (gCode_line[0] == "G02") or (gCode_line[0] == "G03"):
                msg = self.move_handler(gCode_line).split(':')
                # print(msg)
                if len(msg) <= 3:
                    msg1 = f'30 {msg[0]} {msg[2]} {self.feedrate} 3'    # midPoint
                    msg2 = f'30 {msg[1]} {msg[2]} {self.feedrate} 4'    # endPoint
                    print(msg1,msg2)
                    self.put_send(msg1)
                    sleep(0.2)
                    self.put_send(msg2)
                    print(msg1,msg2)
                else:
                    msg1 = f'30 {msg[0]} {msg[4]} {self.feedrate} 3'  # midPoint
                    msg2 = f'30 {msg[1]} {msg[4]} {self.feedrate} 4'  # endPoint
                    msg3 = f'30 {msg[2]} {msg[4]} {self.feedrate} 3'  # midPoint
                    msg4 = f'30 {msg[3]} {msg[4]} {self.feedrate} 4'  # midPoint
                    self.put_send(msg1)
                    sleep(0.5)
                    self.put_send(msg2)
                    sleep(0.5)
                    self.put_send(msg3)
                    sleep(0.5)
                    self.put_send(msg4)
                    sleep(0.5)
    
            elif gCode_line[0] == "G17":
                # self.orientation = f'0.00000 0.00000 1.00000 0.00000'                       # -normala v xy ravnino
                self.plane = 1                                                                # 1; xy ravnina
                self.instructionCompleted = True
    
            elif gCode_line[0] == "G18":
                # self.orientation = f'{str(sqrt(0.5))} {str(sqrt(0.5))} 0.00000 0.00000'     # -normala v xz ravnino
                self.plane = 2                                                                # 2; xz ravnina
                self.instructionCompleted = True
    
            elif gCode_line[0] == "G19":
                # self.orientation = f'{str(sqrt(0.5))} 0.00000 {str(sqrt(0.5))} 0.00000'     # -normala v yz ravnino
                self.plane = 3                                                                # 3; yz ravnina
                self.instructionCompleted = True
    
            elif gCode_line[0] == "G20":
                self.conversion = 25.4
    
            elif gCode_line[0] == "G21":
                self.conversion = 1
                self.instructionCompleted = True
                self.receive_handler(None)
    
            elif gCode_line[0] == "G28":
                if len(gCode_line) > 1:
                    msg = self.move_handler(gCode_line).split(':')
                    msg = f'30 {msg[0]} {msg[1]} {self.feedrate} 2'
                    self.put_send(msg)
                msg = '30 '
                for i in range(len(HOME) - 1):
                    if i < 3:
                        val = str(float(HOME[i]) - float(HOME_OFFSET[i])) + ' '
                    else:
                        val = HOME[i]
                    msg += val
                msg += f' {self.feedrate} 2'
                # print(msg)
                self.put_send(msg)

            elif gCode_line[0] == "G90":
                self.instructionCompleted = True
                self.receive_handler(None)

            #elif gCode_line[0] == "G91":
            #    pass

            #elif gCode_line[0] == "M00":    # program stop emergency stop on flex pendant
            #    pass

            elif gCode_line[0] == "M02":    # end of program
                self.tool_on = False
                msg = '30 '
                for i in range(len(HOME) - 1):
                    if i < 3:
                        val = str(float(HOME[i]) - float(HOME_OFFSET[i])) + ' '
                    else:
                        val = HOME[i]
                    msg += val
                msg += f' {self.feedrate} 2'
                # print(msg)
                self.put_send(msg)

            elif gCode_line[0] == "M03":
                msg = f'30 0 0 0 0 0 0 0 {self.feedrate} 5'
                self.put_send(msg)
                self.instructionCompleted = True
                self.receive_handler(None)

            elif gCode_line[0] == "M05":
                msg = f'30 0 0 0 0 0 0 0 {self.feedrate} 6'
                self.put_send(msg)
                self.instructionCompleted = True
                self.receive_handler(None)
        self.popup.close()
        sleep(0.1)
        msg = '33'      # execute moves in buffer command
        self.put_send(msg)

    def move_handler(self, gCode_line):
        if len(gCode_line) == 3:
                str_midPoint = self.moveCcircle(gCode_line)
                str_endPoint = f'{self.previousGCodePosition[0]} {self.previousGCodePosition[1]} {self.previousGCodePosition[2]}'
        else:
            self.x = gCode_line[1][1::]                                 # gets x coordinates
            self.y = gCode_line[2][1::]                                 # gets y coordinates
            self.z = gCode_line[3][1::]                                 # gets z coordinates
            str_endPoint = self.x + ' ' + self.y + ' ' + self.z
            str_midPoint = ''

            if gCode_line[0] == 'G01' or gCode_line[0] == 'G28':
                if len(gCode_line) == 5:
                    self.feedrate = gCode_line[4][1::]                  # if feedrate then change it else not
                    #self.change_speed()
            elif gCode_line[0] == 'G02' or gCode_line[0] == 'G03':
                if len(gCode_line) == 7: # če ima g code na koncu presledek je to problem
                    self.feedrate = gCode_line[6][1::]
                    #self.change_speed()
                if self.plane == 1:
                    offset = [float(gCode_line[4][1::]), float(gCode_line[5][1::]), 0.0]    # z koordinata nima offseta
                elif self.plane == 2:
                    offset = [float(gCode_line[4][1::]), 0.0, float(gCode_line[5][1::])]
                elif self.plane == 3:
                    offset = [0.0, float(gCode_line[4][1::]), float(gCode_line[5][1::])]
                endPoint = [float(self.x), float(self.y), float(self.z)]
                if gCode_line[0] == 'G02':
                    direction = 'clockwise'
                elif gCode_line[0] == 'G03':
                    direction ='-clockwise'
                str_midPoint = self.moveC(offset, endPoint, direction)

        if len(str_midPoint) > 0:
            msg = f'{str_midPoint}:{str_endPoint}'
        else:
            msg = f'{str_endPoint}'
        self.previousGCodePosition = [float(self.x), float(self.y), float(self.z)]       # saves endpoints
        return f'{msg}:{self.orientation}'

    def moveC(self, centerCoord, finCoord, direction):
        midCoords = ''
        x_0, y_0, z_0 = self.previousGCodePosition
        x_c, y_c, z_c = centerCoord[0] + x_0, centerCoord[1] + y_0, centerCoord[2] + z_0
        x_1, y_1, z_1 = finCoord
        if self.plane == 1:
            r = sqrt((x_c - x_0) ** 2 + (y_c - y_0) ** 2)
            k = sqrt((x_0 - x_1) ** 2 + (y_0 - y_1) ** 2) / 2
            a = asin(k / r)
            fi_1 = atan2((y_1 - y_c), (x_1 - x_c))
            fi_0 = atan2((y_0 - y_c), (x_0 - x_c))
            if fi_0 < 0:
                fi_0 += 2*pi
            if fi_1 < 0:
                fi_1 += 2*pi
            if direction == 'clockwise':
                if fi_0-fi_1 < 0:
                    fi_0 += 2*pi
                if fi_0-fi_1 < pi:
                    midCoords = f'{(x_c + cos(fi_1 + a) * r):.3f} {(y_c + sin(fi_1 + a) * r):.3f} {str(z_0)}'
                else:
                    b = (fi_0-fi_1)/4
                    midCoords = f'{(x_c + cos(fi_0 - b) * r):.3f} {(y_c + sin(fi_0 - b) * r):.3f} {str(z_0)}:\
{(x_c + cos(fi_0 - 2*b) * r):.3f} {(y_c + sin(fi_0 - 2*b) * r):.3f} {str(z_0)}:\
{(x_c + cos(fi_1 + b) * r):.3f} {(y_c + sin(fi_1 + b) * r):.3f} {str(z_0)}'

            elif direction == '-clockwise':
                if fi_1-fi_0 < 0:
                    fi_1 += 2*pi
                if fi_1-fi_0 < pi:
                    midCoords = f'{(x_c + cos(fi_1 - a) * r):.3f} {(y_c + sin(fi_1 - a) * r):.3f} {str(z_0)}'
                else:
                    b = (fi_1-fi_0)/4
                    midCoords = f'{(x_c + cos(fi_0 + b) * r):.3f} {(y_c + sin(fi_0 + b) * r):.3f} {str(z_0)}:\
{(x_c + cos(fi_0 + 2*b) * r):.3f} {(y_c + sin(fi_0 + 2*b) * r):.3f} {str(z_0)}:\
{(x_c + cos(fi_1 - b) * r):.3f} {(y_c + sin(fi_1 - b) * r):.3f} {str(z_0)}'


        elif self.plane == 2:
            r = sqrt((x_c - x_0) ** 2 + (z_c - z_0) ** 2)
            k = sqrt((x_0 - x_1) ** 2 + (z_0 - z_1) ** 2) / 2
            a = asin(k / r)
            fi_1 = atan2((z_1 - z_c), (x_1 - x_c))
            fi_0 = atan2((z_0 - z_c), (x_0 - x_c))
            if fi_0 < 0:
                fi_0 += 2 * pi
            if fi_1 < 0:
                fi_1 += 2 * pi
            if direction == 'clockwise':
                if fi_0 - fi_1 < 0:
                    fi_0 += 2 * pi
                if fi_0 - fi_1 < pi:
                    midCoords = f'{(x_c + cos(fi_1 + a) * r):.3f} {str(y_0)} {(z_c + sin(fi_1 + a) * r):.3f}'
                else:
                    b = (fi_0 - fi_1) / 4
                    midCoords = f'{(x_c + cos(fi_0 - b) * r):.3f} {str(y_0)} {(z_c + sin(fi_0 - b) * r):.3f}:\
{(x_c + cos(fi_0 - 2 * b) * r):.3f} {str(y_0)} {(z_c + sin(fi_0 - 2 * b) * r):.3f}:\
{(x_c + cos(fi_1 + b) * r):.3f} {str(y_0)} {(z_c + sin(fi_1 + b) * r):.3f}'

            elif direction == '-clockwise':
                if fi_1 - fi_0 < 0:
                    fi_1 += 2 * pi
                if fi_1 - fi_0 < pi:
                    midCoords = f'{(x_c + cos(fi_1 - a) * r):.3f} {str(y_0)} {(z_c + sin(fi_1 - a) * r):.3f}'
                else:
                    b = (fi_1 - fi_0) / 4
                    midCoords = f'{(x_c + cos(fi_0 + b) * r):.3f} {str(y_0)} {(z_c + sin(fi_0 + b) * r):.3f}:\
{(x_c + cos(fi_0 + 2 * b) * r):.3f} {str(y_0)} {(z_c + sin(fi_0 + 2 * b) * r):.3f}:\
{(x_c + cos(fi_1 - b) * r):.3f} {str(y_0)} {(z_c + sin(fi_1 - b) * r):.3f}'


        elif self.plane == 3:
            r = sqrt((y_c - y_0) ** 2 + (z_c - z_0) ** 2)
            k = sqrt((y_0 - y_1) ** 2 + (z_0 - z_1) ** 2) / 2
            a = asin(k / r)
            fi_1 = atan2((z_1 - z_c), (y_1 - y_c))
            fi_0 = atan2((z_0 - z_c), (y_0 - y_c))
            if fi_0 < 0:
                fi_0 += 2 * pi
            if fi_1 < 0:
                fi_1 += 2 * pi
            if direction == 'clockwise':
                if fi_0 - fi_1 < 0:
                    fi_0 += 2 * pi
                if fi_0 - fi_1 < pi:
                    midCoords = f'{str(x_0)} {(y_c + cos(fi_1 + a) * r):.3f} {(z_c + sin(fi_1 + a) * r):.3f}'
                else:
                    b = (fi_0 - fi_1) / 4
                    midCoords = f'{str(x_0)} {(y_c + cos(fi_0 - b) * r):.3f} {(z_c + sin(fi_0 - b) * r):.3f}:\
{str(x_0)} {(y_c + cos(fi_0 - 2 * b) * r):.3f} {(z_c + sin(fi_0 - 2 * b) * r):.3f}:\
{str(x_0)} {(y_c + cos(fi_1 + b) * r):.3f} {(z_c + sin(fi_1 + b) * r):.3f}'

            elif direction == '-clockwise':
                if fi_1 - fi_0 < 0:
                    fi_1 += 2 * pi
                if fi_1 - fi_0 < pi:
                    midCoords = f'{str(x_0)} {(y_c + cos(fi_1 - a) * r):.3f} {(z_c + sin(fi_1 - a) * r):.3f}'
                else:
                    b = (fi_1 - fi_0) / 4
                    midCoords = f'{str(x_0)} {(y_c + cos(fi_0 + b) * r):.3f} {(z_c + sin(fi_0 + b) * r):.3f}:\
{str(x_0)} {(y_c + cos(fi_0 + 2 * b) * r):.3f} {(z_c + sin(fi_0 + 2 * b) * r):.3f}:\
{str(x_0)} {(y_c + cos(fi_1 - b) * r):.3f} {(z_c + sin(fi_1 - b) * r):.3f}'
        else:
             print('Incorrect direction argument')
             midCoords = ''
        self.plane == 1
        return midCoords

    def moveCcircle(self, gCode_line):
        instr, st_off, nd_off = gCode_line[0], float(gCode_line[1][1::]), float(gCode_line[2][1::])
        r = sqrt(st_off**2+nd_off**2)
        x_0, y_0, z_0 = self.previousGCodePosition[0], self.previousGCodePosition[1], self.previousGCodePosition[2]
        if self.plane == 1:
            x_c = x_0 + st_off
            y_c = y_0 + nd_off
            z_c = z_0
            fi = atan2(y_0-y_c, x_0-x_c)
            if instr == "G02":
                midCoords = f'{(x_c + cos(fi-pi/2) * r):.3f} {(y_c + sin(fi-pi/2) * r):.3f} {str(z_c)}:\
{(x_c + cos(fi-pi) * r):.3f} {(y_c + sin(fi-pi) * r):.3f} {str(z_c)}:\
{(x_c + cos(fi-3*pi/2) * r):.3f} {(y_c + sin(fi-3*pi/2) * r):.3f} {str(z_c)}'

            elif instr == "G03":
                midCoords = f'{(x_c + cos(fi+pi/2) * r):.3f} {(y_c + sin(fi+pi/2) * r):.3f} {str(z_c)}:\
{(x_c + cos(fi+pi) * r):.3f} {(y_c + sin(fi+pi) * r):.3f} {str(z_c)}:\
{(x_c + cos(fi+3*pi/2) * r):.3f} {(y_c + sin(fi+3*pi/2) * r):.3f} {str(z_c)}'

        elif self.plane == 2:
            x_c = x_0 + st_off
            y_c = y_0
            z_c = z_0 + nd_off
            fi = atan2(z_0 - z_c, x_0 - x_c)
            if instr == "G02":
                midCoords = f'{(x_c + cos(fi - pi / 2) * r):.3f} {str(y_c)} {(z_c + sin(fi - pi / 2) * r):.3f}:\
{(x_c + cos(fi - pi) * r):.3f} {str(y_c)} {(z_c + sin(fi - pi) * r):.3f}:\
{(x_c + cos(fi - 3 * pi / 2) * r):.3f} {str(y_c)} {(z_c + sin(fi - 3 * pi / 2) * r):.3f}'

            elif instr == "G03":
                midCoords = f'{(x_c + cos(fi + pi / 2) * r):.3f} {str(y_c)} {(z_c + sin(fi + pi / 2) * r):.3f}:\
{(x_c + cos(fi + pi) * r):.3f} {str(y_c)} {(z_c + sin(fi + pi) * r):.3f}:\
{(x_c + cos(fi + 3 * pi / 2) * r):.3f} {str(y_c)} {(z_c + sin(fi + 3 * pi / 2) * r):.3f}'
        elif self.plane == 3:
            x_c = x_0
            y_c = y_0 + st_off
            z_c = z_0 + nd_off
            fi = atan2(z_0 - z_c, y_0 - y_c)
            if instr == "G02":
                midCoords = f'{str(x_c)} {(y_c + cos(fi - pi / 2) * r):.3f} {(z_c + sin(fi - pi / 2) * r):.3f}:\
{str(x_c)} {(y_c + cos(fi - pi) * r):.3f} {(z_c + sin(fi - pi) * r):.3f}:\
{str(x_c)} {(y_c + cos(fi - 3 * pi / 2) * r):.3f} {(z_c + sin(fi - 3 * pi / 2) * r):.3f}'

            elif instr == "G03":
                midCoords = f'{str(x_c)} {(y_c + cos(fi + pi / 2) * r):.3f} {(z_c + sin(fi + pi / 2) * r):.3f}:\
{str(x_c)} {(y_c + cos(fi + pi) * r):.3f} {(z_c + sin(fi + pi) * r):.3f}:\
{str(x_c)} {(y_c + cos(fi + 3 * pi / 2) * r):.3f} {(z_c + sin(fi + 3 * pi / 2) * r):.3f}'
        return midCoords

    def change_speed(self, speed):
        if not self.connected:
            return
        msg = f'08 {speed} 50.00 0.0 0.00'
        self.put_send(msg)

    def change_zone(self, zone):
        if not self.connected:
            return
        zone_dict = {'z0': [.3, .3, .03],
                     'z1': [1, 1, .1],
                     'z5': [5, 8, .8],
                     'z10': [10, 15, 1.5],
                     'z15': [15, 23, 2.3],
                     'z20': [20, 30, 3],
                     'z30': [30, 45, 4.5],
                     'z50': [50, 75, 7.5],
                     'z100': [100, 150, 15],
                     'z200': [200, 300, 30]}
        msg = '09 '
        if zone == 'fine':
            msg += '1 0 0 0'
        elif zone in zone_dict.keys():
            zone = zone_dict[zone]
            msg += f'0 {zone[0]} {zone[1]} {zone[2]}'
        self.put_send(msg)

    def init_wobj_tool(self, wobj, tool):
        wobj_msg = f'07 {wobj[0]} {wobj[1]} {wobj[2]} {wobj[3]}' \
                   f' {wobj[4]} {wobj[5]} {wobj[6]}'
        self.put_send(wobj_msg)
        sleep(1)
        tool_msg = f'06 {tool[0]} {tool[1]} {tool[2]} {tool[3]}' \
                   f' {tool[4]} {tool[5]} {tool[6]}'
        self.put_send(tool_msg)



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
        if self.connected:
            self.init_wobj_tool(WOBJ_INIT, TOOL_INIT)

    # ===============receive====================== #

    def receive(self):
        while True:
            if self.connected:
                # self.btn_send.setEnabled(True)
                self.btn_send_gCode.setEnabled(True)
                self.btn_set_tool_wobj.setEnabled(True)
                self.btn_file_explorer.setEnabled(True)
                self.btn_connect.setText('Disconnect')
            elif not self.connected:
                # self.btn_send.setEnabled(False)
                self.btn_send_gCode.setEnabled(False)
                self.btn_set_tool_wobj.setEnabled(False)
                self.btn_file_explorer.setEnabled(False)
                self.btn_connect.setText('Connect\n to Robot')
            try:
                data = self.client_socket.recv(BUFFER)
                data = data.decode('utf-8')
                if data is not None:
                    if data.find('#') != -1:
                        start = timer()
                        self.receive_handler(data)
                        data = None
            except:
                try:
                    if timer()-start > 2 and self.connected:
                        self.connected = False
                        self.tool_on = False
                        self.indicator_connection.setStyleSheet("background-color: red")
                        self.indicator_tool.setStyleSheet("background-color: red")
                        self.client_socket.shutdown(socket.SHUT_RDWR)
                        self.client_socket.close()
                except:
                    pass
                continue


    def receive_handler(self, data):
        # print(data)
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
        elif data[0] == '!':
            coord = data.split(' ')
            self.label_x_pos.setText(coord[1])
            self.label_y_pos.setText(coord[2])
            self.label_z_pos.setText(coord[3])
            self.label_q1.setText(coord[4])
            self.label_q2.setText(coord[5])
            self.label_q3.setText(coord[6])
            self.label_q4.setText(coord[7])
        elif data[0] == '$':
            coord = data.split(' ')
            self.previousGCodePosition = [coord[1], coord[2], coord[3]]
        elif data == 'TT#':
            self.tool_on = True
            self.indicator_tool.setStyleSheet("background-color: green")
        elif data == 'TF#':
            self.tool_on = False
            self.indicator_tool.setStyleSheet("background-color: red")
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
        global HOME_OFFSET
        if self.checkBox_wobj.isChecked():
            wobj = [str(self.input_wobj_x.value()),
                    str(self.input_wobj_y.value()),
                    str(self.input_wobj_z.value()),
                    str(self.input_wobj_q1.value()),
                    str(self.input_wobj_q2.value()),
                    str(self.input_wobj_q3.value()),
                    str(self.input_wobj_q4.value())]
            wobj_msg = f'07 {wobj[0]} {wobj[1]} {wobj[2]} {wobj[3]} {wobj[4]} {wobj[5]} {wobj[6]}'
            HOME_OFFSET = [wobj[0], wobj[1], wobj[2]]
            self.x.put_send(wobj_msg)
            sleep(0.1)

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
            sleep(0.1)

    def confirm(self):
        self.apply_values()
        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    wnd = Window()
    wnd.show()
    sys.exit(app.exec_())
