import serial
from serial.tools import list_ports
import time
import datetime as dt
from math import *
import csv
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys
import numpy as np
import sys
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("pygame not available. Xbox controller functionality will be disabled.")

def find_available_ports():
    connected_ports = []
    for port, desc, hwid in list_ports.comports():
        try:
            with serial.Serial(port, timeout=1) as ser:
                if ser.readable():
                    connected_ports.append(port)
        except serial.SerialException as e:
            pass
    return connected_ports


class MainWindow(QWidget):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
                
        self.setWindowTitle('Catheter Robot Control Interface')
        self.setMinimumSize(1920, 1080)
        self.overall_fontsize = 24
        self.label_style = {"font-size": "36px"}
        self.title_style = {"color": "black", "font-size": "36px"}
        self.plot_label_style = {"font-size": "20px"}
        self.connected_ports = find_available_ports()

        self.t_0      = 0 # Initial time
        self.t_prev   = 0 # Previuos time
        self.t        = 0 # Current time
    
        self.t_teensy = 0 # Time in the Teensy 4.1

        self.Connection_Flag    = False
        self.LogginButton_Flag  = False
        self.Data_Received_Flag = 0
        self.first_teensy_time  = True
        self.Control_Mode_Flag = 0 # 0 = Direct Motor Control | 1 = Task Space Control
        self.Rotation_Button_Clicked = False
        self.motor_extra_cmd = 0

        # Gear parameters
        self.r_G1 = 8 # radius [8 mm] (17 teeth) This is the gear attached to the rotation motion motor (M1)
        self.r_G2 = 30 # radius [30 mm] (60 teeth) This is the gear that rotatates the whole inner mechanism
        self.r_G3 = 32 # radius [32 mm] (64 teeth) This is the gear attached to the insertion motor (M2)
        self.r_G4 = 10 # radius [10 mm] (20 teeth) This is the gear that drives the bevel gears (Bevel gears are 10 mm Diam)
        self.d_Roller = 20 # diameter [20 mm] This is the diameter of the rollers moving (forward/backward) the catheter
        # Expressions 
        # Catheter Rotation: theta_G2 = (r_G1/r_G2)*theta_M1
        # Catheter Translation: l_T = PI*d_Roller*(r_G3/r_G4)*theta_M2
        # NOTE: To decouple translation from rotation: theta_M2 = (r_G1/r_G2)*theta_M1
        #################

        self.fps = 100
        self.Plots_RefreshRate = round(1000/self.fps)
        self.Teensy_freq_ble   = 100
        self.plot_time_window  = 10
        self.buffer_size  = round(self.plot_time_window * self.Teensy_freq_ble)
        self.t_buffer = list([0] *round(self.buffer_size))
        # Instrument 1
        ## I1 Translation
        self.I1_T_Motor_pos      = 0
        self.I1_T_Motor_posref   = 0
        self.I1_T_Effector_pos    = 0
        self.I1_T_Effector_posref = 0
        self.I1_T_Motor_pos_buff       = self.t_buffer.copy()
        self.I1_T_Motor_posref_buff    = self.t_buffer.copy()
        self.I1_T_Effector_pos_buff    = self.t_buffer.copy()
        self.I1_T_Effector_posref_buff = self.t_buffer.copy()
        ## I1 Rotation
        self.I1_R_Motor_pos      = 0
        self.I1_R_Motor_posref   = 0
        self.I1_R_Effector_pos    = 0
        self.I1_R_Effector_posref = 0
        self.I1_R_Motor_pos_buff      = self.t_buffer.copy()
        self.I1_R_Motor_posref_buff   = self.t_buffer.copy()
        self.I1_R_Effector_pos_buff    = self.t_buffer.copy()
        self.I1_R_Effector_posref_buff = self.t_buffer.copy()

        # Instrument 2
        ## I2 Translation
        self.I2_T_Motor_pos      = 0
        self.I2_T_Motor_posref   = 0
        self.I2_T_Effector_pos    = 0
        self.I2_T_Effector_posref = 0
        self.I2_T_Motor_pos_buff      = self.t_buffer.copy()
        self.I2_T_Motor_posref_buff   = self.t_buffer.copy()
        self.I2_T_Effector_pos_buff    = self.t_buffer.copy()
        self.I2_T_Effector_posref_buff = self.t_buffer.copy()
        ## I2 Rotation
        self.I2_R_Motor_pos      = 0
        self.I2_R_Motor_posref   = 0
        self.I2_R_Effector_pos    = 0
        self.I2_R_Effector_posref = 0
        self.I2_R_Motor_pos_buff      = self.t_buffer.copy()
        self.I2_R_Motor_posref_buff   = self.t_buffer.copy()
        self.I2_R_Effector_pos_buff    = self.t_buffer.copy()
        self.I2_R_Effector_posref_buff = self.t_buffer.copy()
        # Instrument 3
        ## I3 Translation
        self.I3_T_Motor_pos      = 0
        self.I3_T_Motor_posref   = 0
        self.I3_T_Effector_pos    = 0
        self.I3_T_Effector_posref = 0
        self.I3_T_Motor_pos_buff      = self.t_buffer.copy()
        self.I3_T_Motor_posref_buff   = self.t_buffer.copy()
        self.I3_T_Effector_pos_buff    = self.t_buffer.copy()
        self.I3_T_Effector_posref_buff = self.t_buffer.copy()
        ## I3 Rotation
        self.I3_R_Motor_pos      = 0
        self.I3_R_Motor_posref   = 0
        self.I3_R_Effector_pos    = 0
        self.I3_R_Effector_posref = 0
        self.I3_R_Motor_pos_buff      = self.t_buffer.copy()
        self.I3_R_Motor_posref_buff   = self.t_buffer.copy()
        self.I3_R_Effector_pos_buff    = self.t_buffer.copy()
        self.I3_R_Effector_posref_buff = self.t_buffer.copy()

        self.red   = pg.mkPen(color=(255, 0, 0), width = 2)
        self.blue  = pg.mkPen(color=(0, 0, 255), width = 2)
        self.green = pg.mkPen(color=(0, 255, 0), width = 2)

        # Layout def
        MainLayout         = QVBoxLayout() # Window Layout
        self.Comm_Layout   = QHBoxLayout() # Layout for the COM pot selection and the Log data button
        self.Instruments_Layout = QHBoxLayout() # Control mode Layout

        # Initialize UI elements
        self.Create_DevicesCombobox()
        self.Create_ConnectButton()
        self.Create_LoggingButton()
        self.Create_StopMotorButton()
        self.Create_ResetMotorButton()
        self.Create_CtrlMode_Block()
        self.Create_DMC_Instrument_1_Block()
        self.Create_DMC_Instrument_2_Block()
        self.Create_DMC_Instrument_3_Block()

        # Add widgets to layouts
        # Communications Layout
        self.stretch_factor = 10
        self.Device_Label = QLabel("Device:")
        self.Device_Label.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.Comm_Layout.addWidget(self.Device_Label)
        self.Comm_Layout.addWidget(self.DevicesCombobox, stretch=self.stretch_factor)
        self.Comm_Layout.addWidget(self.ConnectButton, stretch=self.stretch_factor)
        self.Comm_Layout.addWidget(self.LoggingButton, stretch=self.stretch_factor)

        MainLayout.addLayout(self.Comm_Layout)
        MainLayout.addWidget(self.CtrlMode_Block)
        MainLayout.addLayout(self.Instruments_Layout)
        self.Instruments_Layout.addWidget(self.Instrument_1_Block)
        self.Instruments_Layout.addWidget(self.Instrument_2_Block)
        self.Instruments_Layout.addWidget(self.Instrument_3_Block)
        self.setLayout(MainLayout)

    # Creation of the timer for executing the function
        self.timer = QtCore.QTimer()
        self.timer.setInterval(self.Plots_RefreshRate)
        self.timer.timeout.connect(self.all)
        self.timer.start()

    def all(self):
        if self.Connection_Flag:
            if self.ser.in_waiting > 0:
                self.ConnectButton.setText("Receiving Data")
                self.ConnectButton.setStyleSheet("background-color: green;"
                                                 "color: white;"
                                                 "font: Bold;"
                                                 f"font-size: {self.overall_fontsize}px;")
                self.Receive_data()
                if self.Data_Received_Flag:
                    self.Connection_Flag = True
                    self.LoggingButton.setEnabled(True)
                    self.LoggingButton.setStyleSheet("background-color: rgb(0, 0, 170);"
                                                    "color: white;"
                                                    "font: Bold;"
                                                    f"font-size: {self.overall_fontsize}px;")
                    self.StopMotorButton.setEnabled(True)
                    self.StopMotorButton.setStyleSheet("background-color: rgb(255, 0, 0);"
                                                    "color: white;"
                                                    "font: Bold;"
                                                    f"font-size: {self.overall_fontsize}px;")
                    self.ResetMotorButton.setEnabled(True)
                    self.ResetMotorButton.setStyleSheet("background-color: rgb(255, 119, 0);"
                                                "color: white;"
                                                "font: Bold;"
                                                f"font-size: {self.overall_fontsize}px;")
                    self.CtrlJoint_Rbutton.setEnabled(True)
                    # self.CtrlSpd_Rbutton.setEnabled(True)
                    # self.CtrlTorque_Rbutton.setEnabled(True)
                    # self.CtrlHybrid_Rbutton.setEnabled(True)
                    self.CtrlTask_Rbutton.setEnabled(True)
                    self.update_plot_data()
                    self.Data_Received_Flag = 0
            # self.update_plot_data()

    # Function to connect to the selected device
    def Connect_Button_Clicked(self):
        ### Defining the size of the received packages ###
        self.RXD_datapackage_length = 32 # Receive data package size
        self.TXD_datapackage_length = 20 # Transmit data package size
        self.data_length        = self.RXD_datapackage_length - 3
        self.decoded_data       = [0]*self.data_length
        self.BAUD_RATE          = 115200
        self.SER_PORT           = self.DevicesCombobox.currentText()

        self.ser = serial.Serial(port = self.SER_PORT, baudrate = self.BAUD_RATE)
        self.ser.timeout = 0 # set read timeout
        if self.SER_PORT:
            print(f"Connecting to {self.SER_PORT}...")
            print(self.ser)
            if self.ser.is_open:
                print(f'{self.SER_PORT} port opened')
                self.ConnectButton.setText("Connecting...")
                self.ConnectButton.setStyleSheet("background-color: green;"
                                                 "color: white;"
                                                 "font: Bold;"
                                                 f"font-size: {self.overall_fontsize}px;")
                self.Connection_Flag = True

    def StopMotor_Button_Clicked(self):
        print("Stop button")
        self.motor_extra_cmd = 0x81
        self.Transmit_data()
        self.motor_extra_cmd = 0

    def ResetMotor_Button_Clicked(self):
        print("Reset button")
        self.motor_extra_cmd = 0x76
        self.Transmit_data()
        self.motor_extra_cmd = 0

    def Receive_data(self):
        if self.ser.in_waiting >= self.RXD_datapackage_length:
            if self.ser.read(1) == b'\xA5':  # 165 in uint8
                if self.ser.read(1) == b'\x5A':  # 90 in uint8
                    self.expected_length = self.ser.read(1)
                    if self.expected_length == bytes([self.RXD_datapackage_length]):
                        if self.ser.in_waiting >= self.data_length:
                            coded_data = self.ser.read(self.data_length)

                            if self.Control_Mode_Flag == 1 or self.Control_Mode_Flag == 4 or self.Control_Mode_Flag == 5:
                                scaling_factor = 10.0
                            elif self.Control_Mode_Flag == 2:
                                scaling_factor = 1
                            else:
                                scaling_factor = 100.0

                            decode_i = 0
                            for i in range(1, self.data_length, 2):
                                var = coded_data[i-1] + coded_data[i] * 256
                                var = (var - 65536) / scaling_factor if var > 32767 else var / scaling_factor
                                self.decoded_data[decode_i] = var
                                decode_i += 1
                        
                            self.t_teensy       = self.decoded_data[0]*(scaling_factor/10.0)
                            self.I1_T_Motor_pos = self.decoded_data[1]
                            self.I1_R_Motor_pos = self.decoded_data[2]
                            self.I2_T_Motor_pos = self.decoded_data[3]
                            self.I2_R_Motor_pos = self.decoded_data[4]
                            self.I3_T_Motor_pos = self.decoded_data[5]
                            self.I3_R_Motor_pos = self.decoded_data[6]
                            self.Data_Received_Flag = coded_data[25]
                            
                            if self.first_teensy_time:
                                self.t_0_teensy = self.t_teensy
                                self.first_teensy_time = False

    def Transmit_data(self):
        # Check if connection is established
        if not hasattr(self, 'TXD_datapackage_length') or not hasattr(self, 'ser'):
            print("Warning: Serial connection not established. Cannot transmit data.")
            return
        
        # Check if serial port is open
        if not hasattr(self, 'ser') or not self.ser.is_open:
            print("Warning: Serial port is not open. Cannot transmit data.")
            return

        I1_M1_cmd = int(self.I1_T_Motor_posref)
        I1_M2_cmd = int(self.I1_R_Motor_posref)
        I2_M1_cmd = int(self.I2_T_Motor_posref)
        I2_M2_cmd = int(self.I2_R_Motor_posref)
        I3_M1_cmd = int(self.I3_T_Motor_posref)
        I3_M2_cmd = int(self.I3_R_Motor_posref)
    
        # Create signed 16-bit values in big-endian format to match Teensy expectations
        I1_M1_bytes = I1_M1_cmd.to_bytes(2, byteorder='big', signed=True)
        I1_M2_bytes = I1_M2_cmd.to_bytes(2, byteorder='big', signed=True)
        I2_M1_bytes = I2_M1_cmd.to_bytes(2, byteorder='big', signed=True)
        I2_M2_bytes = I2_M2_cmd.to_bytes(2, byteorder='big', signed=True)
        I3_M1_bytes = I3_M1_cmd.to_bytes(2, byteorder='big', signed=True)
        I3_M2_bytes = I3_M2_cmd.to_bytes(2, byteorder='big', signed=True)

        self.data_package = bytearray([165, 90, self.TXD_datapackage_length,
                                       I1_M1_bytes[0], I1_M1_bytes[1],
                                       I1_M2_bytes[0], I1_M2_bytes[1],
                                       I2_M1_bytes[0], I2_M1_bytes[1],
                                       I2_M2_bytes[0], I2_M2_bytes[1],
                                       I3_M1_bytes[0], I3_M1_bytes[1],
                                       I3_M2_bytes[0], I3_M2_bytes[1],
                                       self.Control_Mode_Flag, self.motor_extra_cmd, 0, 0, 0])
        
        if self.ser.is_open:
            self.ser.write(self.data_package)

    def update_plot_data(self):
        if self.Connection_Flag == True:

            self.t = time.time() - self.t_0

            if self.t_prev != self.t:
                # This must be improved
                if self.Rotation_Button_Clicked:
                    self.I1_T_Effector_pos = self.I1_T_Effector_pos
                    self.I1_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I1_R_Motor_pos)
                    self.I2_T_Effector_pos = self.I2_T_Effector_pos
                    self.I2_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I2_R_Motor_pos)
                    self.I3_T_Effector_pos = self.I3_T_Effector_pos
                    self.I3_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I3_R_Motor_pos)
                else:
                    self.I1_T_Effector_pos = self.Translation_Transform_M2_to_Catheter(self.I1_T_Motor_pos)
                    self.I1_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I1_R_Motor_pos)
                    self.I2_T_Effector_pos = self.Translation_Transform_M2_to_Catheter(self.I2_T_Motor_pos)
                    self.I2_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I2_R_Motor_pos)
                    self.I3_T_Effector_pos = self.Translation_Transform_M2_to_Catheter(self.I3_T_Motor_pos)
                    self.I3_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I3_R_Motor_pos)
                # 
                self.current_time = self.t_teensy - self.t_0_teensy
                self.buffers = [self.t_buffer,
                                self.I1_T_Motor_pos_buff, self.I1_T_Motor_posref_buff,
                                self.I1_R_Motor_pos_buff, self.I1_R_Motor_posref_buff,
                                self.I2_T_Motor_pos_buff, self.I2_T_Motor_posref_buff,
                                self.I2_R_Motor_pos_buff, self.I2_R_Motor_posref_buff,
                                self.I3_T_Motor_pos_buff, self.I3_T_Motor_posref_buff,
                                self.I3_R_Motor_pos_buff, self.I3_R_Motor_posref_buff,
                                self.I1_T_Effector_pos_buff, self.I1_T_Effector_posref_buff,
                                self.I1_R_Effector_pos_buff, self.I1_R_Effector_posref_buff,
                                self.I2_T_Effector_pos_buff, self.I2_T_Effector_posref_buff,
                                self.I2_R_Effector_pos_buff, self.I2_R_Effector_posref_buff,
                                self.I3_T_Effector_pos_buff, self.I3_T_Effector_posref_buff,
                                self.I3_R_Effector_pos_buff, self.I3_R_Effector_posref_buff]
                self.values  = [self.current_time,
                                self.I1_T_Motor_pos, self.I1_T_Motor_posref/10,
                                self.I1_R_Motor_pos, self.I1_R_Motor_posref/10,
                                self.I2_T_Motor_pos, self.I2_T_Motor_posref/10,
                                self.I2_R_Motor_pos, self.I2_R_Motor_posref/10,
                                self.I3_T_Motor_pos, self.I3_T_Motor_posref/10,
                                self.I3_R_Motor_pos, self.I3_R_Motor_posref/10,
                                self.I1_T_Effector_pos, self.I1_T_Effector_posref/10,
                                self.I1_R_Effector_pos, self.I1_R_Effector_posref/10,
                                self.I2_T_Effector_pos, self.I2_T_Effector_posref/10,
                                self.I2_R_Effector_pos, self.I2_R_Effector_posref/10,
                                self.I3_T_Effector_pos, self.I3_T_Effector_posref/10,
                                self.I3_R_Effector_pos, self.I3_R_Effector_posref/10]
                self.update_buffers(self.buffers, self.values)

                # Instrument 1
                ## Translation
                self.I1_T_Motor_pos_line.setData(self.t_buffer, self.I1_T_Motor_pos_buff)
                self.I1_T_Effector_pos_line.setData(self.t_buffer, self.I1_T_Effector_pos_buff)
                self.I1_T_Block_Label.setText(str(self.I1_T_Motor_pos))
                ## Rotation
                self.I1_R_Motor_pos_line.setData(self.t_buffer, self.I1_R_Motor_pos_buff)
                self.I1_R_Effector_pos_line.setData(self.t_buffer, self.I1_R_Effector_pos_buff)                
                self.I1_R_Block_Label.setText(str(self.I1_R_Motor_pos))

                # Instrument 2
                ## Translation
                self.I2_T_Motor_pos_line.setData(self.t_buffer, self.I2_T_Motor_pos_buff)
                self.I2_T_Effector_pos_line.setData(self.t_buffer, self.I2_T_Effector_pos_buff)
                self.I2_T_Block_Label.setText(str(self.I2_T_Motor_pos))
                ## Rotation
                self.I2_R_Motor_pos_line.setData(self.t_buffer, self.I2_R_Motor_pos_buff)
                self.I2_R_Effector_pos_line.setData(self.t_buffer, self.I2_R_Effector_pos_buff)
                self.I2_R_Block_Label.setText(str(self.I2_R_Motor_pos))

                # Instrument 3
                ## Translation
                self.I3_T_Motor_pos_line.setData(self.t_buffer, self.I3_T_Motor_pos_buff)
                self.I3_T_Effector_pos_line.setData(self.t_buffer, self.I3_T_Effector_pos_buff)
                self.I3_T_Block_Label.setText(str(self.I3_T_Motor_pos))
                ## Rotation
                self.I3_R_Motor_pos_line.setData(self.t_buffer, self.I3_R_Motor_pos_buff)
                self.I3_R_Effector_pos_line.setData(self.t_buffer, self.I3_R_Effector_pos_buff)
                self.I3_R_Block_Label.setText(str(self.I3_R_Motor_pos))

                if self.Control_Mode_Flag == 5:
                    self.I1_T_cmd_line.setData(self.t_buffer, self.I1_T_Effector_posref_buff)
                    self.I1_R_cmd_line.setData(self.t_buffer, self.I1_R_Effector_posref_buff)
                    self.I2_T_cmd_line.setData(self.t_buffer, self.I2_T_Effector_posref_buff)
                    self.I2_R_cmd_line.setData(self.t_buffer, self.I2_R_Effector_posref_buff)
                    self.I3_T_cmd_line.setData(self.t_buffer, self.I3_T_Effector_posref_buff)
                    self.I3_R_cmd_line.setData(self.t_buffer, self.I3_R_Effector_posref_buff)
                else:
                    self.I1_T_cmd_line.setData(self.t_buffer, self.I1_T_Motor_posref_buff)
                    self.I1_R_cmd_line.setData(self.t_buffer, self.I1_R_Motor_posref_buff)
                    self.I2_T_cmd_line.setData(self.t_buffer, self.I2_T_Motor_posref_buff)
                    self.I2_R_cmd_line.setData(self.t_buffer, self.I2_R_Motor_posref_buff)
                    self.I3_T_cmd_line.setData(self.t_buffer, self.I3_T_Motor_posref_buff)
                    self.I3_R_cmd_line.setData(self.t_buffer, self.I3_R_Motor_posref_buff)

                if self.LogginButton_Flag == True:
                    self.LoggedData = {
                        "time": self.t,                        
                        "M1_Pos_cmd": self.M1_pos_cmd
                    }

                    with open(self.csv_file_name, mode="a", newline="") as self.file:
                        self.writer = csv.DictWriter(self.file, fieldnames = self.DataHeaders)
                        self.writer.writerow(self.LoggedData)
            self.t_prev = self.t
        else:
            print("NOT Connected")

    def update_buffers(self, buffers, values):
        for buffer, value in zip(buffers, values):
            buffer[:] = buffer[1:]  # This modifies the original buffer
            buffer.append(value)

    def LogginButton_Clicked(self):

        self.LogginButton_Flag = True

        self.t_0 = time.time()
        self.LoggingButton.setText("Logging data")
        self.LoggingButton.setStyleSheet("background-color : blue")
        self.csv_file_name = "GUI_Logger_" + time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
        self.DataHeaders   = ["time", "M1_Pos", "M2_Pos", "M3_Pos", "M4_Pos", "M5_Pos"]

        with open(self.csv_file_name, mode="w", newline="") as self.file:
            self.writer = csv.DictWriter(self.file, fieldnames = self.DataHeaders)
            self.writer.writeheader()

    def Create_DevicesCombobox(self):
        self.DevicesCombobox = QComboBox()
        self.DevicesCombobox.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.DevicesCombobox.addItems(self.connected_ports)
    
    def Create_ConnectButton(self):
        self.ConnectButton = QPushButton("Connect")
        self.ConnectButton.setStyleSheet("background-color: rgb(150, 150, 150);"
                                    "color: white;"
                                    "font: Bold;"
                                    f"font-size: {self.overall_fontsize}px;")
        self.ConnectButton.clicked.connect(self.Connect_Button_Clicked)

    def Create_StopMotorButton(self):
        self.StopMotorButton = QPushButton("Stop Motors")
        self.StopMotorButton.setEnabled(False)
        self.StopMotorButton.setStyleSheet("background-color: rgb(150, 150, 150);"
                                    "color: white;"
                                    "font: Bold;"
                                    f"font-size: {self.overall_fontsize}px;")
        self.StopMotorButton.clicked.connect(self.StopMotor_Button_Clicked)

    def Create_ResetMotorButton(self):
        self.ResetMotorButton = QPushButton("Reset Motors")
        self.ResetMotorButton.setEnabled(False)
        self.ResetMotorButton.setStyleSheet("background-color: rgb(150, 150, 150);"
                                    "color: white;"
                                    "font: Bold;"
                                    f"font-size: {self.overall_fontsize}px;")
        self.ResetMotorButton.clicked.connect(self.ResetMotor_Button_Clicked)

    def Create_LoggingButton(self):
        self.LoggingButton = QPushButton("Log Data")
        self.LoggingButton.setEnabled(False)
        self.LoggingButton.setStyleSheet("background-color: rgb(150, 150, 150);"
                                    "color: white;"
                                    "font: Bold;"
                                    f"font-size: {self.overall_fontsize}px;")
        self.LoggingButton.clicked.connect(self.LogginButton_Clicked)

    def Create_CtrlMode_Block(self):
        ## Level of Assistance Block
        self.CtrlMode_Block  = QGroupBox("Control Mode")
        self.CtrlMode_Block.setAlignment(Qt.AlignCenter)
        self.CtrlMode_Block.setStyleSheet(f"font-size: 28px;")
        self.CtrlMode_Block_Layout = QHBoxLayout()
        self.CtrlMode_Block.setLayout(self.CtrlMode_Block_Layout)
        self.CtrlMode_Block.setEnabled(True)
        # Control Modes
        self.CtrlJoint_Rbutton  = QRadioButton("Position Motor Control")
        # self.CtrlSpd_Rbutton    = QRadioButton("Speed Motor Control")
        # self.CtrlTorque_Rbutton = QRadioButton("Torque Motor Control")
        # self.CtrlHybrid_Rbutton = QRadioButton("Hybrid Position-Force Motor Control")
        self.CtrlTask_Rbutton   = QRadioButton("Task Space Control")
        self.CtrlXbox_Rbutton  = QRadioButton("Xbox Controller Control")
        self.CtrlMode_Block_Layout.addWidget(self.StopMotorButton)
        self.CtrlMode_Block_Layout.addWidget(self.ResetMotorButton)
        self.CtrlMode_Block_Layout.addWidget(self.CtrlJoint_Rbutton)
        # self.CtrlMode_Block_Layout.addWidget(self.CtrlSpd_Rbutton)
        # self.CtrlMode_Block_Layout.addWidget(self.CtrlTorque_Rbutton)
        # self.CtrlMode_Block_Layout.addWidget(self.CtrlHybrid_Rbutton)
        self.CtrlMode_Block_Layout.addWidget(self.CtrlTask_Rbutton)
        self.CtrlMode_Block_Layout.addWidget(self.CtrlXbox_Rbutton)
        self.CtrlJoint_Rbutton.setEnabled(False)
        # self.CtrlSpd_Rbutton.setEnabled(False)
        # self.CtrlTorque_Rbutton.setEnabled(False)
        # self.CtrlHybrid_Rbutton.setEnabled(False)
        self.CtrlTask_Rbutton.setEnabled(False)
        self.CtrlXbox_Rbutton.setEnabled(True)
        self.CtrlJoint_Rbutton.toggled.connect(self.Direct_Motor_Ctrl_mode)
        # self.CtrlSpd_Rbutton.toggled.connect(self.Speed_Ctrl_mode)
        # self.CtrlTorque_Rbutton.toggled.connect(self.Torque_Ctrl_mode)
        # self.CtrlHybrid_Rbutton.toggled.connect(self.Hybrid_Ctrl_mode)
        self.CtrlTask_Rbutton.toggled.connect(self.Task_Space_Ctrl_mode)
        self.CtrlXbox_Rbutton.toggled.connect(self.Xbox_Ctrl_mode)

    def Create_DMC_Instrument_1_Block(self):
        """Creates the complete Instrument 1 control block with Translation and Rotation sections"""
        
        # === Main Instrument 1 Container ===
        self.Instrument_1_Block = QGroupBox("Instrument 1")
        self.Instrument_1_Block.setAlignment(Qt.AlignCenter)
        self.Instrument_1_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.Instrument_1_Block_Layout = QVBoxLayout()
        self.Instrument_1_Block.setLayout(self.Instrument_1_Block_Layout)
        self.Instrument_1_Block.setEnabled(False)
        
        # === I1 Translation/Insertion Block ===
        self.I1_T_Block = QGroupBox("Insertion / Withdrawal")
        self.I1_T_Block.setAlignment(Qt.AlignCenter)
        self.I1_T_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I1_T_Block_Layout = QGridLayout()
        self.I1_T_Block.setLayout(self.I1_T_Block_Layout)
        self.I1_T_Block.setEnabled(True)
        
        # Translation Control Buttons
        self.I1_T_upbtn = QPushButton("^")
        self.I1_T_downbtn = QPushButton("v")
        self.I1_T_Block_Label = QLabel(f"deg")
        
        # Translation Scale Selection ComboBox
        self.I1_T_sclebox = QComboBox()
        self.I1_T_sclebox.addItem("x 10")
        self.I1_T_sclebox.addItem("x 1")
        self.I1_T_sclebox.addItem("x 0.1")
        self.I1_T_sclebox.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I1_T_sclebox.setCurrentIndex(1)  # Default to "x 1"
        
        # Translation Plot Widget
        self.I1_Translation_Motor_Plot = pg.PlotWidget()
        self.I1_Translation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I1_Translation_Motor_Plot.setLabel('left', "[deg]", **self.plot_label_style)
        self.I1_Translation_Motor_Plot.setLabel('bottom', "Time [s]", **self.plot_label_style)
        self.I1_Translation_Motor_Plot.addLegend()
        self.I1_Translation_Motor_Plot.setBackground('w')
        self.I1_Translation_Motor_Plot.showGrid(x=True, y=True)
        
        # Translation Plot Lines
        self.I1_T_cmd_line = self.I1_Translation_Motor_Plot.plot(self.t_buffer, self.I1_T_Motor_posref_buff, name="Command", pen=self.blue)
        self.I1_T_Motor_pos_line = self.I1_Translation_Motor_Plot.plot(self.t_buffer, self.I1_T_Motor_pos_buff, name="Motor", pen=self.red)
        self.I1_T_Effector_pos_line = self.I1_Translation_Motor_Plot.plot(self.t_buffer, self.I1_T_Effector_pos_buff, name="Catheter", pen=self.green)
        
        # Layout Translation Controls (Grid positions)
        self.I1_T_Block_Layout.addWidget(self.I1_T_upbtn, 1, 1)
        self.I1_T_Block_Layout.addWidget(self.I1_T_Block_Label, 2, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I1_T_Block_Layout.addWidget(self.I1_T_downbtn, 3, 1)
        self.I1_T_Block_Layout.addWidget(self.I1_T_sclebox, 4, 0, 1, 2)
        self.I1_T_Block_Layout.addWidget(self.I1_Translation_Motor_Plot, 0, 2, 5, 5)
        
        # Connect Translation Button Events
        self.I1_T_upbtn.clicked.connect(self.I1_T_UpBtn_Clicked)
        self.I1_T_downbtn.clicked.connect(self.I1_T_DownBtn_Clicked)
        
        # === I1 Rotation Block ===
        self.I1_R_Block = QGroupBox("Rotation")
        self.I1_R_Block.setAlignment(Qt.AlignCenter)
        self.I1_R_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I1_R_Block_Layout = QGridLayout()
        self.I1_R_Block.setLayout(self.I1_R_Block_Layout)
        self.I1_R_Block.setEnabled(True)
        
        # Rotation Control Buttons
        self.I1_R_SineBtn = QPushButton("Send Sine")
        self.I1_R_SineBtn.setStyleSheet("font-size: 18px;")
        self.I1_R_upbtn = QPushButton("^")
        self.I1_R_downbtn = QPushButton("v")
        self.I1_R_Block_Label = QLabel(f"deg")
        
        # Rotation Scale Selection ComboBox
        self.I1_R_sclebox = QComboBox()
        self.I1_R_sclebox.addItem("x 10")
        self.I1_R_sclebox.addItem("x 1")
        self.I1_R_sclebox.addItem("x 0.1")
        self.I1_R_sclebox.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I1_R_sclebox.setCurrentIndex(1)  # Default to "x 1"
        
        # Rotation Plot Widget
        self.I1_Rotation_Motor_Plot = pg.PlotWidget()
        self.I1_Rotation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I1_Rotation_Motor_Plot.setLabel('left', "[deg]", **self.plot_label_style)
        self.I1_Rotation_Motor_Plot.setLabel('bottom', "Time [s]", **self.plot_label_style)
        self.I1_Rotation_Motor_Plot.addLegend()
        self.I1_Rotation_Motor_Plot.setBackground('w')
        self.I1_Rotation_Motor_Plot.showGrid(x=True, y=True)
        
        # Rotation Plot Lines
        self.I1_R_cmd_line = self.I1_Rotation_Motor_Plot.plot(self.t_buffer, self.I1_R_Motor_posref_buff, name="Command", pen=self.blue)
        self.I1_R_Motor_pos_line = self.I1_Rotation_Motor_Plot.plot(self.t_buffer, self.I1_R_Motor_pos_buff, name="Motor", pen=self.red)
        self.I1_R_Effector_pos_line = self.I1_Rotation_Motor_Plot.plot(self.t_buffer, self.I1_R_Effector_pos_buff, name="Catheter", pen=self.green)
        
        # Layout Rotation Controls (Grid positions)
        self.I1_R_Block_Layout.addWidget(self.I1_R_SineBtn, 0, 0, 2, 2)
        self.I1_R_Block_Layout.addWidget(self.I1_R_upbtn, 1, 1)
        self.I1_R_Block_Layout.addWidget(self.I1_R_Block_Label, 2, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I1_R_Block_Layout.addWidget(self.I1_R_downbtn, 3, 1)
        self.I1_R_Block_Layout.addWidget(self.I1_R_sclebox, 4, 0, 1, 2)
        self.I1_R_Block_Layout.addWidget(self.I1_Rotation_Motor_Plot, 0, 2, 5, 5)
        
        # Connect Rotation Button Events
        self.I1_R_SineBtn.clicked.connect(self.I1_R_SendSineBtn_Clicked)
        self.I1_R_upbtn.clicked.connect(self.I1_R_UpBtn_Clicked)
        self.I1_R_downbtn.clicked.connect(self.I1_R_DownBtn_Clicked)
        
        # === Assemble Main Instrument Block ===
        self.Instrument_1_Block_Layout.addWidget(self.I1_T_Block)
        self.Instrument_1_Block_Layout.addWidget(self.I1_R_Block)

    def Create_DMC_Instrument_2_Block(self):
        """Creates the complete Instrument 2 control block with Translation and Rotation sections"""

        # === Main Instrument 2 Container ===
        self.Instrument_2_Block = QGroupBox("Instrument 2")
        self.Instrument_2_Block.setAlignment(Qt.AlignCenter)
        self.Instrument_2_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.Instrument_2_Block_Layout = QVBoxLayout()
        self.Instrument_2_Block.setLayout(self.Instrument_2_Block_Layout)
        self.Instrument_2_Block.setEnabled(False)

        # === I2 Translation/Insertion Block ===
        self.I2_T_Block = QGroupBox("Insertion / Withdrawal")
        self.I2_T_Block.setAlignment(Qt.AlignCenter)
        self.I2_T_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I2_T_Block_Layout = QGridLayout()
        self.I2_T_Block.setLayout(self.I2_T_Block_Layout)
        self.I2_T_Block.setEnabled(True)

        # Translation Control Buttons
        self.I2_T_upbtn = QPushButton("^")
        self.I2_T_downbtn = QPushButton("v")
        self.I2_T_Block_Label = QLabel(f"deg")

        # Translation Scale Selection ComboBox
        self.I2_T_sclebox = QComboBox()
        self.I2_T_sclebox.addItem("x 10")
        self.I2_T_sclebox.addItem("x 1")
        self.I2_T_sclebox.addItem("x 0.1")
        self.I2_T_sclebox.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I2_T_sclebox.setCurrentIndex(1)  # Default to "x 1"

        # Translation Plot Widget
        self.I2_Translation_Motor_Plot = pg.PlotWidget()
        self.I2_Translation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I2_Translation_Motor_Plot.setLabel('left', "[deg]", **self.plot_label_style)
        self.I2_Translation_Motor_Plot.setLabel('bottom', "Time [s]", **self.plot_label_style)
        self.I2_Translation_Motor_Plot.addLegend()
        self.I2_Translation_Motor_Plot.setBackground('w')
        self.I2_Translation_Motor_Plot.showGrid(x=True, y=True)

        # Translation Plot Lines
        self.I2_T_cmd_line = self.I2_Translation_Motor_Plot.plot(self.t_buffer, self.I2_T_Motor_posref_buff, name="Command", pen=self.blue)
        self.I2_T_Motor_pos_line = self.I2_Translation_Motor_Plot.plot(self.t_buffer, self.I2_T_Motor_pos_buff, name="Motor", pen=self.red)
        self.I2_T_Effector_pos_line = self.I2_Translation_Motor_Plot.plot(self.t_buffer, self.I2_T_Effector_pos_buff, name="Catheter", pen=self.green)

        # Layout Translation Controls (Grid positions)
        self.I2_T_Block_Layout.addWidget(self.I2_T_upbtn, 1, 1)
        self.I2_T_Block_Layout.addWidget(self.I2_T_Block_Label, 2, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I2_T_Block_Layout.addWidget(self.I2_T_downbtn, 3, 1)
        self.I2_T_Block_Layout.addWidget(self.I2_T_sclebox, 4, 0, 1, 2)
        self.I2_T_Block_Layout.addWidget(self.I2_Translation_Motor_Plot, 0, 2, 5, 5)

        # Connect Translation Button Events
        self.I2_T_upbtn.clicked.connect(self.I2_T_UpBtn_Clicked)
        self.I2_T_downbtn.clicked.connect(self.I2_T_DownBtn_Clicked)

        # === I2 Rotation Block ===
        self.I2_R_Block = QGroupBox("Rotation")
        self.I2_R_Block.setAlignment(Qt.AlignCenter)
        self.I2_R_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I2_R_Block_Layout = QGridLayout()
        self.I2_R_Block.setLayout(self.I2_R_Block_Layout)
        self.I2_R_Block.setEnabled(True)

        # Rotation Control Buttons
        self.I2_R_SineBtn = QPushButton("Send Sine")
        self.I2_R_SineBtn.setStyleSheet("font-size: 18px;")
        self.I2_R_upbtn = QPushButton("^")
        self.I2_R_downbtn = QPushButton("v")
        self.I2_R_Block_Label = QLabel(f"deg")

        # Rotation Scale Selection ComboBox
        self.I2_R_sclebox = QComboBox()
        self.I2_R_sclebox.addItem("x 10")
        self.I2_R_sclebox.addItem("x 1")
        self.I2_R_sclebox.addItem("x 0.1")
        self.I2_R_sclebox.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I2_R_sclebox.setCurrentIndex(1)  # Default to "x 1"

        # Rotation Plot Widget
        self.I2_Rotation_Motor_Plot = pg.PlotWidget()
        self.I2_Rotation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I2_Rotation_Motor_Plot.setLabel('left', "[deg]", **self.plot_label_style)
        self.I2_Rotation_Motor_Plot.setLabel('bottom', "Time [s]", **self.plot_label_style)
        self.I2_Rotation_Motor_Plot.addLegend()
        self.I2_Rotation_Motor_Plot.setBackground('w')
        self.I2_Rotation_Motor_Plot.showGrid(x=True, y=True)

        # Rotation Plot Lines
        self.I2_R_cmd_line = self.I2_Rotation_Motor_Plot.plot(self.t_buffer, self.I2_R_Motor_posref_buff, name="Command", pen=self.blue)
        self.I2_R_Motor_pos_line = self.I2_Rotation_Motor_Plot.plot(self.t_buffer, self.I2_R_Motor_pos_buff, name="Motor", pen=self.red)
        self.I2_R_Effector_pos_line = self.I2_Rotation_Motor_Plot.plot(self.t_buffer, self.I2_R_Effector_pos_buff, name="Catheter", pen=self.green)

        # Layout Rotation Controls (Grid positions)
        self.I2_R_Block_Layout.addWidget(self.I2_R_SineBtn, 0, 0, 2, 2)
        self.I2_R_Block_Layout.addWidget(self.I2_R_upbtn, 1, 1)
        self.I2_R_Block_Layout.addWidget(self.I2_R_Block_Label, 2, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I2_R_Block_Layout.addWidget(self.I2_R_downbtn, 3, 1)
        self.I2_R_Block_Layout.addWidget(self.I2_R_sclebox, 4, 0, 1, 2)
        self.I2_R_Block_Layout.addWidget(self.I2_Rotation_Motor_Plot, 0, 2, 5, 5)

        # Connect Rotation Button Events
        self.I2_R_SineBtn.clicked.connect(self.I2_R_SendSineBtn_Clicked)
        self.I2_R_upbtn.clicked.connect(self.I2_R_UpBtn_Clicked)
        self.I2_R_downbtn.clicked.connect(self.I2_R_DownBtn_Clicked)

        # === Assemble Main Instrument Block ===
        self.Instrument_2_Block_Layout.addWidget(self.I2_T_Block)
        self.Instrument_2_Block_Layout.addWidget(self.I2_R_Block)

    def Create_DMC_Instrument_3_Block(self):
        """Creates the complete Instrument 3 control block with Translation and Rotation sections"""

        # === Main Instrument 3 Container ===
        self.Instrument_3_Block = QGroupBox("Instrument 3")
        self.Instrument_3_Block.setAlignment(Qt.AlignCenter)
        self.Instrument_3_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.Instrument_3_Block_Layout = QVBoxLayout()
        self.Instrument_3_Block.setLayout(self.Instrument_3_Block_Layout)
        self.Instrument_3_Block.setEnabled(False)

        # === I3 Translation/Insertion Block ===
        self.I3_T_Block = QGroupBox("Insertion / Withdrawal")
        self.I3_T_Block.setAlignment(Qt.AlignCenter)
        self.I3_T_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I3_T_Block_Layout = QGridLayout()
        self.I3_T_Block.setLayout(self.I3_T_Block_Layout)
        self.I3_T_Block.setEnabled(True)

        # Translation Control Buttons
        self.I3_T_upbtn = QPushButton("^")
        self.I3_T_downbtn = QPushButton("v")
        self.I3_T_Block_Label = QLabel(f"deg")

        # Translation Scale Selection ComboBox
        self.I3_T_sclebox = QComboBox()
        self.I3_T_sclebox.addItem("x 10")
        self.I3_T_sclebox.addItem("x 1")
        self.I3_T_sclebox.addItem("x 0.1")
        self.I3_T_sclebox.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I3_T_sclebox.setCurrentIndex(1)  # Default to "x 1"

        # Translation Plot Widget
        self.I3_Translation_Motor_Plot = pg.PlotWidget()
        self.I3_Translation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I3_Translation_Motor_Plot.setLabel('left', "[deg]", **self.plot_label_style)
        self.I3_Translation_Motor_Plot.setLabel('bottom', "Time [s]", **self.plot_label_style)
        self.I3_Translation_Motor_Plot.addLegend()
        self.I3_Translation_Motor_Plot.setBackground('w')
        self.I3_Translation_Motor_Plot.showGrid(x=True, y=True)

        # Translation Plot Lines
        self.I3_T_cmd_line = self.I3_Translation_Motor_Plot.plot(self.t_buffer, self.I3_T_Motor_posref_buff, name="Command", pen=self.blue)
        self.I3_T_Motor_pos_line = self.I3_Translation_Motor_Plot.plot(self.t_buffer, self.I3_T_Motor_pos_buff, name="Motor", pen=self.red)
        self.I3_T_Effector_pos_line = self.I3_Translation_Motor_Plot.plot(self.t_buffer, self.I3_T_Effector_pos_buff, name="Catheter", pen=self.green)

        # Layout Translation Controls (Grid positions)
        self.I3_T_Block_Layout.addWidget(self.I3_T_upbtn, 1, 1)
        self.I3_T_Block_Layout.addWidget(self.I3_T_Block_Label, 2, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I3_T_Block_Layout.addWidget(self.I3_T_downbtn, 3, 1)
        self.I3_T_Block_Layout.addWidget(self.I3_T_sclebox, 4, 0, 1, 2)
        self.I3_T_Block_Layout.addWidget(self.I3_Translation_Motor_Plot, 0, 2, 5, 5)

        # Connect Translation Button Events
        self.I3_T_upbtn.clicked.connect(self.I3_T_UpBtn_Clicked)
        self.I3_T_downbtn.clicked.connect(self.I3_T_DownBtn_Clicked)

        # === I3 Rotation Block ===
        self.I3_R_Block = QGroupBox("Rotation")
        self.I3_R_Block.setAlignment(Qt.AlignCenter)
        self.I3_R_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I3_R_Block_Layout = QGridLayout()
        self.I3_R_Block.setLayout(self.I3_R_Block_Layout)
        self.I3_R_Block.setEnabled(True)

        # Rotation Control Buttons
        self.I3_R_SineBtn = QPushButton("Send Sine")
        self.I3_R_SineBtn.setStyleSheet("font-size: 18px;")
        self.I3_R_upbtn = QPushButton("^")
        self.I3_R_downbtn = QPushButton("v")
        self.I3_R_Block_Label = QLabel(f"deg")

        # Rotation Scale Selection ComboBox
        self.I3_R_sclebox = QComboBox()
        self.I3_R_sclebox.addItem("x 10")
        self.I3_R_sclebox.addItem("x 1")
        self.I3_R_sclebox.addItem("x 0.1")
        self.I3_R_sclebox.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I3_R_sclebox.setCurrentIndex(1)  # Default to "x 1"

        # Rotation Plot Widget
        self.I3_Rotation_Motor_Plot = pg.PlotWidget()
        self.I3_Rotation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I3_Rotation_Motor_Plot.setLabel('left', "[deg]", **self.plot_label_style)
        self.I3_Rotation_Motor_Plot.setLabel('bottom', "Time [s]", **self.plot_label_style)
        self.I3_Rotation_Motor_Plot.addLegend()
        self.I3_Rotation_Motor_Plot.setBackground('w')
        self.I3_Rotation_Motor_Plot.showGrid(x=True, y=True)

        # Rotation Plot Lines
        self.I3_R_cmd_line = self.I3_Rotation_Motor_Plot.plot(self.t_buffer, self.I3_R_Motor_posref_buff, name="Command", pen=self.blue)
        self.I3_R_Motor_pos_line = self.I3_Rotation_Motor_Plot.plot(self.t_buffer, self.I3_R_Motor_pos_buff, name="Motor", pen=self.red)
        self.I3_R_Effector_pos_line = self.I3_Rotation_Motor_Plot.plot(self.t_buffer, self.I3_R_Effector_pos_buff, name="Catheter", pen=self.green)

        # Layout Rotation Controls (Grid positions)
        self.I3_R_Block_Layout.addWidget(self.I3_R_SineBtn, 0, 0, 2, 2)
        self.I3_R_Block_Layout.addWidget(self.I3_R_upbtn, 1, 1)
        self.I3_R_Block_Layout.addWidget(self.I3_R_Block_Label, 2, 1, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I3_R_Block_Layout.addWidget(self.I3_R_downbtn, 3, 1)
        self.I3_R_Block_Layout.addWidget(self.I3_R_sclebox, 4, 0, 1, 2)
        self.I3_R_Block_Layout.addWidget(self.I3_Rotation_Motor_Plot, 0, 2, 5, 5)

        # Connect Rotation Button Events
        self.I3_R_SineBtn.clicked.connect(self.I3_R_SendSineBtn_Clicked)
        self.I3_R_upbtn.clicked.connect(self.I3_R_UpBtn_Clicked)
        self.I3_R_downbtn.clicked.connect(self.I3_R_DownBtn_Clicked)

        # === Assemble Main Instrument Block ===
        self.Instrument_3_Block_Layout.addWidget(self.I3_T_Block)
        self.Instrument_3_Block_Layout.addWidget(self.I3_R_Block)

    # Instrument 1 Buttons
    def I1_T_UpBtn_Clicked(self):
        self.Rotation_Button_Clicked = False
        if self.Control_Mode_Flag == 5:
            self.I1_T_Effector_posref = self.Compute_Increment(self.I1_T_Effector_posref, self.I1_T_sclebox)
            self.I1_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I1_T_Effector_posref)
        else:
            self.I1_T_Motor_posref = self.Compute_Increment(self.I1_T_Motor_posref, self.I1_T_sclebox)
        self.Transmit_data()

    def I1_T_DownBtn_Clicked(self):
        self.Rotation_Button_Clicked = False
        if self.Control_Mode_Flag == 5:
            self.I1_T_Effector_posref = self.Compute_Decrement(self.I1_T_Effector_posref, self.I1_T_sclebox)
            self.I1_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I1_T_Effector_posref)
        else:
            self.I1_T_Motor_posref = self.Compute_Decrement(self.I1_T_Motor_posref, self.I1_T_sclebox)
        self.Transmit_data()

    def I1_R_UpBtn_Clicked(self):
        self.CW_Rotation = True
        self.CCW_Rotation = False
        self.Rotation_Button_Clicked = True
        if self.Control_Mode_Flag == 5:
            self.I1_R_Effector_posref = self.Compute_Increment(self.I1_R_Effector_posref, self.I1_R_sclebox)
            self.I1_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I1_R_Effector_posref)
        else:
            self.I1_R_Motor_posref = self.Compute_Increment(self.I1_R_Motor_posref, self.I1_R_sclebox)
        self.I1_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I1_R_Motor_posref, self.I1_T_Motor_posref, self.I1_R_sclebox) # This is to compensate the coupling
        self.Transmit_data()

    def I1_R_DownBtn_Clicked(self):
        self.CW_Rotation = False
        self.CCW_Rotation = True
        self.Rotation_Button_Clicked = True
        if self.Control_Mode_Flag == 5:
            self.I1_R_Effector_posref = self.Compute_Decrement(self.I1_R_Effector_posref, self.I1_R_sclebox)
            self.I1_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I1_R_Effector_posref)
        else:
            self.I1_R_Motor_posref = self.Compute_Decrement(self.I1_R_Motor_posref, self.I1_R_sclebox)
        self.I1_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I1_R_Motor_posref, self.I1_T_Motor_posref, self.I1_R_sclebox) # This is to compensate the coupling
        self.Transmit_data()

    def I1_R_SendSineBtn_Clicked(self):
        self.I1_R_Effector_posref = self.Generate_Sine_Wave(self.I1_R_Effector_posref)
        self.I1_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I1_R_Effector_posref)
        self.Transmit_data()


 # Instrument 2 Buttons
    def I2_T_UpBtn_Clicked(self):
        self.Rotation_Button_Clicked = False
        if self.Control_Mode_Flag == 5:
            self.I2_T_Effector_posref = self.Compute_Increment(self.I2_T_Effector_posref, self.I2_T_sclebox)
            self.I2_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I2_T_Effector_posref)
        else:
            self.I2_T_Motor_posref = self.Compute_Increment(self.I2_T_Motor_posref, self.I2_T_sclebox)
        self.Transmit_data()

    def I2_T_DownBtn_Clicked(self):
        self.Rotation_Button_Clicked = False
        if self.Control_Mode_Flag == 5:
            self.I2_T_Effector_posref = self.Compute_Decrement(self.I2_T_Effector_posref, self.I2_T_sclebox)
            self.I2_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I2_T_Effector_posref)
        else:
            self.I2_T_Motor_posref = self.Compute_Decrement(self.I2_T_Motor_posref, self.I2_T_sclebox)
        self.Transmit_data()

    def I2_R_UpBtn_Clicked(self):
        self.CW_Rotation = True
        self.CCW_Rotation = False
        self.Rotation_Button_Clicked = True
        if self.Control_Mode_Flag == 5:
            self.I2_R_Effector_posref = self.Compute_Increment(self.I2_R_Effector_posref, self.I2_R_sclebox)
            self.I2_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I2_R_Effector_posref)
        else:
            self.I2_R_Motor_posref = self.Compute_Increment(self.I2_R_Motor_posref, self.I2_R_sclebox)
        self.I2_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I2_R_Motor_posref, self.I2_T_Motor_posref, self.I2_R_sclebox) # This is to compensate the coupling
        self.Transmit_data()

    def I2_R_DownBtn_Clicked(self):
        self.CW_Rotation = False
        self.CCW_Rotation = True
        self.Rotation_Button_Clicked = True
        if self.Control_Mode_Flag == 5:
            self.I2_R_Effector_posref = self.Compute_Decrement(self.I2_R_Effector_posref, self.I2_R_sclebox)
            self.I2_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I2_R_Effector_posref)
        else:
            self.I2_R_Motor_posref = self.Compute_Decrement(self.I2_R_Motor_posref, self.I2_R_sclebox)
        self.I2_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I2_R_Motor_posref, self.I2_T_Motor_posref, self.I2_R_sclebox) # This is to compensate the coupling
        self.Transmit_data()

    def I2_R_SendSineBtn_Clicked(self):
        self.I2_R_Effector_posref = self.Generate_Sine_Wave(self.I2_R_Effector_posref)
        self.I2_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I2_R_Effector_posref)
        self.Transmit_data()


 # Instrument 3 Buttons
    def I3_T_UpBtn_Clicked(self):
        self.Rotation_Button_Clicked = False
        if self.Control_Mode_Flag == 5:
            self.I3_T_Effector_posref = self.Compute_Increment(self.I3_T_Effector_posref, self.I3_T_sclebox)
            self.I3_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I3_T_Effector_posref)
        else:
            self.I3_T_Motor_posref = self.Compute_Increment(self.I3_T_Motor_posref, self.I3_T_sclebox)
        self.Transmit_data()

    def I3_T_DownBtn_Clicked(self):
        self.Rotation_Button_Clicked = False
        if self.Control_Mode_Flag == 5:
            self.I3_T_Effector_posref = self.Compute_Decrement(self.I3_T_Effector_posref, self.I3_T_sclebox)
            self.I3_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I3_T_Effector_posref)
        else:
            self.I3_T_Motor_posref = self.Compute_Decrement(self.I3_T_Motor_posref, self.I3_T_sclebox)
        self.Transmit_data()

    def I3_R_UpBtn_Clicked(self):
        self.CW_Rotation = True
        self.CCW_Rotation = False
        self.Rotation_Button_Clicked = True
        if self.Control_Mode_Flag == 5:
            self.I3_R_Effector_posref = self.Compute_Increment(self.I3_R_Effector_posref, self.I3_R_sclebox)
            self.I3_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I3_R_Effector_posref)
        else:
            self.I3_R_Motor_posref = self.Compute_Increment(self.I3_R_Motor_posref, self.I3_R_sclebox)
        self.I3_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I3_R_Motor_posref, self.I3_T_Motor_posref, self.I3_R_sclebox) # This is to compensate the coupling
        self.Transmit_data()

    def I3_R_DownBtn_Clicked(self):
        self.CW_Rotation = False
        self.CCW_Rotation = True
        self.Rotation_Button_Clicked = True
        if self.Control_Mode_Flag == 5:
            self.I3_R_Effector_posref = self.Compute_Decrement(self.I3_R_Effector_posref, self.I3_R_sclebox)
            self.I3_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I3_R_Effector_posref)
        else:
            self.I3_R_Motor_posref = self.Compute_Decrement(self.I3_R_Motor_posref, self.I3_R_sclebox)
        self.I3_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I3_R_Motor_posref, self.I3_T_Motor_posref, self.I3_R_sclebox) # This is to compensate the coupling
        self.Transmit_data()

    def I3_R_SendSineBtn_Clicked(self):
        self.I3_R_Effector_posref = self.Generate_Sine_Wave(self.I3_R_Effector_posref)
        self.I3_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I3_R_Effector_posref)
        self.Transmit_data()


    # Transformations
    def Rotation_Transform_M1_to_Catheter(self, theta_M1): # Given Motor 1 angular position (responsible for rotation) computes the anglular position & displacement of the catheter
        theta_Catheter = (self.r_G1/self.r_G2)*theta_M1
        return theta_Catheter

    def Rotation_Transform_Catheter_to_M1(self, theta_Catheter): # Given Motor 1 angular position (responsible for rotation) computes the anglular position & displacement of the catheter
        theta_M1 = theta_Catheter/(self.r_G1/self.r_G2)
        return theta_M1
    
    def Translation_Transform_M2_to_Catheter(self, theta_M2): # Given Motor 2 angular position (responsible for rotation) computes the linear position & displacement of the catheter
        Catheter_Translation = pi*self.d_Roller*(self.r_G3/self.r_G4)*self.deg2rad(theta_M2/360)
        return Catheter_Translation
    
    def Translation_Transform_Catheter_to_M2(self, Catheter_Translation): # Given Motor 2 angular position (responsible for rotation) computes the linear position & displacement of the catheter
        # Catheter_Translation = pi*self.d_Roller*(self.r_G3/self.r_G4)*theta_M2
        theta_M2 = self.rad2deg(Catheter_Translation/(pi*self.d_Roller*(self.r_G3/self.r_G4)))
        return theta_M2
    
    def Decouple_Rotation_Translation_M1_to_M2(self, theta_M1, theta_M2, scale_combobox):
        if self.CW_Rotation:
            # cmd_M2 = theta_M2 - (self.r_G1/self.r_G2)*(theta_M1 - self.Compute_Decrement(theta_M1, scale_combobox))
            cmd_M2 = theta_M2 - self.Rotation_Transform_M1_to_Catheter((theta_M1 - self.Compute_Decrement(theta_M1, scale_combobox)))
        else:
            cmd_M2 = theta_M2 - self.Rotation_Transform_M1_to_Catheter((theta_M1 - self.Compute_Increment(theta_M1, scale_combobox)))
        return cmd_M2
        

    def Compute_Increment(self, current_cmd, scale_combobox):
        if scale_combobox.currentText() == "x 0.1":
            Displacement_cmd = current_cmd + 0.1
            return Displacement_cmd
        elif scale_combobox.currentText() == "x 10":
            Displacement_cmd = current_cmd + 10
            return Displacement_cmd
        else:  # "x 1"
            Displacement_cmd = current_cmd + 1
            return Displacement_cmd

    def Compute_Decrement(self, current_cmd, scale_combobox):
        if scale_combobox.currentText() == "x 0.1":
            Displacement_cmd = current_cmd - 0.1
            return Displacement_cmd
        elif scale_combobox.currentText() == "x 10":
            Displacement_cmd = current_cmd - 10
            return Displacement_cmd
        else:  # "x 1"
            Displacement_cmd = current_cmd - 1
            return Displacement_cmd
            


    def Direct_Motor_Ctrl_mode(self):
        self.Instrument_1_Block.setEnabled(True)
        self.Instrument_2_Block.setEnabled(True)
        self.Instrument_3_Block.setEnabled(True)
        self.Control_Mode_Flag = 1
        self.Transmit_data()


    def Task_Space_Ctrl_mode(self):
        self.Instrument_1_Block.setEnabled(True)
        self.Instrument_2_Block.setEnabled(True)
        self.Instrument_3_Block.setEnabled(True)
        self.Control_Mode_Flag  = 5
        self.Transmit_data()

    def Xbox_Ctrl_mode(self):
        self.Instrument_1_Block.setEnabled(False)
        self.Instrument_2_Block.setEnabled(False)
        self.Instrument_3_Block.setEnabled(False)
        self.xbox_window = XboxControllerWindow(main_window=self)
        self.xbox_window.show()
 

    def update_buffers(self, buffers, values):
        for buffer, value in zip(buffers, values):
            buffer[:] = buffer[1:]  # This modifies the original buffer
            buffer.append(value)
 

    def saturation(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    
    def deg2rad(self,deg):
        return deg*pi/180
    
    def rad2deg(self,deg):
        return deg*180/pi

class JoystickWidget(QtWidgets.QWidget):
    """Custom widget to display joystick position with visual feedback"""
    
    def __init__(self, side, parent=None):
        super(JoystickWidget, self).__init__(parent)
        self.side = side
        self.x_pos = 0.0  # Joystick X position (-1 to 1)
        self.y_pos = 0.0  # Joystick Y position (-1 to 1)
        self.setFixedSize(120, 120)
        
    def update_position(self, x, y):
        """Update joystick position and repaint"""
        self.x_pos = max(-1, min(1, x))  # Clamp between -1 and 1
        self.y_pos = max(-1, min(1, y))  # Clamp between -1 and 1
        self.update()  # Trigger repaint
        
    def paintEvent(self, event):
        """Custom paint event to draw joystick visualization"""
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        
        # Widget dimensions
        width = self.width()
        height = self.height()
        center_x = width // 2
        center_y = height // 2
        radius = min(width, height) // 2 - 10
        
        # Draw outer circle (joystick boundary)
        painter.setPen(QtGui.QPen(QtGui.QColor(85, 85, 85), 3))
        painter.setBrush(QtGui.QBrush(QtGui.QColor(26, 26, 26)))
        painter.drawEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)
        
        # Draw center crosshairs
        painter.setPen(QtGui.QPen(QtGui.QColor(100, 100, 100), 1))
        painter.drawLine(center_x - radius + 5, center_y, center_x + radius - 5, center_y)
        painter.drawLine(center_x, center_y - radius + 5, center_x, center_y + radius - 5)
        
        # Calculate joystick thumb position
        thumb_x = center_x + (self.x_pos * (radius - 15))
        thumb_y = center_y + (self.y_pos * (radius - 15))
        
        # Draw joystick thumb (moving indicator)
        thumb_radius = 8
        
        # Choose color based on movement
        if abs(self.x_pos) > 0.1 or abs(self.y_pos) > 0.1:
            # Moving - bright color
            thumb_color = QtGui.QColor(255, 255, 0)  # Yellow
            border_color = QtGui.QColor(255, 255, 255)  # White border
        else:
            # Centered - dim color
            thumb_color = QtGui.QColor(150, 150, 150)  # Gray
            border_color = QtGui.QColor(100, 100, 100)  # Dark gray border
        
        painter.setPen(QtGui.QPen(border_color, 2))
        painter.setBrush(QtGui.QBrush(thumb_color))
        painter.drawEllipse(int(thumb_x - thumb_radius), int(thumb_y - thumb_radius), 
                          thumb_radius * 2, thumb_radius * 2)
        
        # Draw direction indicator line when moving
        if abs(self.x_pos) > 0.1 or abs(self.y_pos) > 0.1:
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 0), 2))
            painter.drawLine(center_x, center_y, int(thumb_x), int(thumb_y))

class XboxControllerWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None, main_window=None):
        super(XboxControllerWindow, self).__init__(parent)
        self.setWindowTitle("Xbox Controller")
        self.setGeometry(100, 100, 1800, 1400)
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)
        
        # Reference to main window for instrument control
        self.main_window = main_window
        
        # Instrument selection state
        self.selected_instrument = None  # None, 1, 2, or 3
        self.instrument_status = {
            1: False,  # Instrument 1 active
            2: False,  # Instrument 2 active  
            3: False   # Instrument 3 active
        }
        
        # Joystick control parameters
        self.joystick_deadzone = 0.1
        self.joystick_sensitivity = 0.5
        self.overall_fontsize = 24
        
        # Xbox controller variables
        self.controller = None
        self.controller_connected = False
        self.available_controllers = []
        
        # Initialize pygame for controller support
        if PYGAME_AVAILABLE:
            pygame.init()
            pygame.joystick.init()
        
        # Add Xbox controller UI elements here
        self.create_controller_connection_ui()
        self.create_xbox_controller_layout()
        
        # Start controller polling timer
        self.controller_timer = QtCore.QTimer()
        self.controller_timer.timeout.connect(self.update_controller_input)
        self.controller_timer.start(50)  # 20 FPS polling rate
        
    def create_controller_connection_ui(self):
        """Create controller connection interface"""
        connection_group = QtWidgets.QGroupBox("Controller Connection")
        connection_group.setStyleSheet("""
            QGroupBox {
                font: bold 16px;
                color: black;
                border: 2px solid #555;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        
        connection_layout = QtWidgets.QHBoxLayout(connection_group)
        
        # Controller selection combobox
        self.controller_combo = QtWidgets.QComboBox()
        self.controller_combo.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        
        # Connect/Disconnect button
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.setStyleSheet(f"""
            QPushButton {{
                background-color: #4CAF50;
                color: white;
                font: bold {self.overall_fontsize}px;
                padding: 10px;
                border: none;
                border-radius: 5px;
            }}
            QPushButton:hover {{
                background-color: #45a049;
            }}
            QPushButton:pressed {{
                background-color: #3d8b40;
            }}
        """)
        self.connect_button.clicked.connect(self.toggle_controller_connection)
        
        # Refresh button
        self.refresh_button = QtWidgets.QPushButton("Refresh")
        self.refresh_button.setStyleSheet(f"""
            QPushButton {{
                background-color: #2196F3;
                color: white;
                font: bold {self.overall_fontsize}px;
                padding: 10px;
                border: none;
                border-radius: 5px;
            }}
            QPushButton:hover {{
                background-color: #1976D2;
            }}
        """)
        self.refresh_button.clicked.connect(self.refresh_controllers)
        
        connection_layout.addWidget(QtWidgets.QLabel("Controller:"))
        connection_layout.addWidget(self.controller_combo)
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.refresh_button)
        
        self.layout.addWidget(connection_group)
        
        # Add instrument status display
        self.create_instrument_status_display()
        
        # Initial controller scan
        self.refresh_controllers()
        
    def create_instrument_status_display(self):
        """Create instrument status display"""
        # Instrument Status Group
        instrument_group = QtWidgets.QGroupBox("Instrument Control Status")
        instrument_group.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        
        instrument_layout = QtWidgets.QHBoxLayout(instrument_group)
        
        # Instrument status labels
        self.instrument_labels = {}
        for i in range(1, 4):
            label = QtWidgets.QLabel(f"Instrument {i}: INACTIVE")
            label.setStyleSheet(f"""
                QLabel {{
                    font: bold {self.overall_fontsize}px;
                    padding: 10px;
                    border: 2px solid #888;
                    border-radius: 5px;
                    background-color: #f0f0f0;
                    color: #666;
                }}
            """)
            self.instrument_labels[i] = label
            instrument_layout.addWidget(label)
        
        # Control instructions
        instructions = QtWidgets.QLabel("Press A/B/X buttons to select instruments. Use joysticks to control.")
        instructions.setStyleSheet(f"font-size: {self.overall_fontsize-4}px; color: #666;")
        instructions.setWordWrap(True)
        
        instrument_layout.addWidget(instructions)
        self.layout.addWidget(instrument_group)
        
    def update_instrument_status(self, instrument_num, active):
        """Update instrument status display"""
        if instrument_num in self.instrument_labels:
            label = self.instrument_labels[instrument_num]
            if active:
                label.setText(f"Instrument {instrument_num}: ACTIVE")
                label.setStyleSheet(f"""
                    QLabel {{
                        font: bold {self.overall_fontsize}px;
                        padding: 10px;
                        border: 2px solid #4CAF50;
                        border-radius: 5px;
                        background-color: #e8f5e8;
                        color: #2e7d32;
                    }}
                """)
            else:
                label.setText(f"Instrument {instrument_num}: INACTIVE")
                label.setStyleSheet(f"""
                    QLabel {{
                        font: bold {self.overall_fontsize}px;
                        padding: 10px;
                        border: 2px solid #888;
                        border-radius: 5px;
                        background-color: #f0f0f0;
                        color: #666;
                    }}
                """)
        
    def refresh_controllers(self):
        """Scan for available Xbox controllers"""
        self.available_controllers = []
        self.controller_combo.clear()
        
        if not PYGAME_AVAILABLE:
            self.controller_combo.addItem("pygame not available")
            return
            
        pygame.joystick.quit()
        pygame.joystick.init()
        
        controller_count = pygame.joystick.get_count()
        
        if controller_count == 0:
            self.controller_combo.addItem("No controllers detected")
        else:
            for i in range(controller_count):
                joystick = pygame.joystick.Joystick(i)
                controller_name = joystick.get_name()
                self.available_controllers.append(i)
                self.controller_combo.addItem(f"Controller {i}: {controller_name}")
                
        print(f"Found {controller_count} controller(s)")
        
    def toggle_controller_connection(self):
        """Connect or disconnect the selected controller"""
        if not PYGAME_AVAILABLE:
            return
            
        if self.controller_connected:
            self.disconnect_controller()
        else:
            self.connect_controller()
            
    def connect_controller(self):
        """Connect to the selected controller"""
        if not self.available_controllers:
            return
            
        selected_index = self.controller_combo.currentIndex()
        if selected_index < 0 or selected_index >= len(self.available_controllers):
            return
            
        controller_id = self.available_controllers[selected_index]
        
        try:
            self.controller = pygame.joystick.Joystick(controller_id)
            self.controller.init()
            self.controller_connected = True
            
            self.connect_button.setText("Disconnect")
            self.connect_button.setStyleSheet(f"""
                QPushButton {{
                    background-color: #f44336;
                    color: white;
                    font: bold {self.overall_fontsize}px;
                    padding: 10px;
                    border: none;
                    border-radius: 5px;
                }}
                QPushButton:hover {{
                    background-color: #da190b;
                }}
            """)
            
            # Update status
            self.connection_status.setText("Status: Connected")
            self.connection_status.setStyleSheet("color: #44ff44; font: bold 12px;")
            
            print(f"Connected to controller: {self.controller.get_name()}")
            
        except Exception as e:
            print(f"Error connecting to controller: {e}")
            
    def disconnect_controller(self):
        """Disconnect the current controller"""
        if self.controller:
            self.controller.quit()
            self.controller = None
            
        self.controller_connected = False
        
        self.connect_button.setText("Connect")
        self.connect_button.setStyleSheet(f"""
            QPushButton {{
                background-color: #4CAF50;
                color: white;
                font: bold {self.overall_fontsize}px;
                padding: 10px;
                border: none;
                border-radius: 5px;
            }}
            QPushButton:hover {{
                background-color: #45a049;
            }}
        """)
        
        # Update status
        self.connection_status.setText("Status: Disconnected")
        self.connection_status.setStyleSheet("color: #ff4444; font: bold 12px;")
        
        print("Controller disconnected")
        
    def update_controller_input(self):
        """Update controller input and UI elements"""
        if not self.controller_connected or not self.controller:
            return
            
        try:
            pygame.event.pump()
            
            # Update joystick values
            left_x = self.controller.get_axis(0)
            left_y = self.controller.get_axis(1)
            right_x = self.controller.get_axis(2)
            right_y = self.controller.get_axis(3)
            
            # Update UI labels
            self.left_x_label.setText(f"Left X: {left_x:.2f}")
            self.left_y_label.setText(f"Left Y: {left_y:.2f}")
            self.right_x_label.setText(f"Right X: {right_x:.2f}")
            self.right_y_label.setText(f"Right Y: {right_y:.2f}")
            
            # Update joystick visual feedback
            self.left_joystick_widget.update_position(left_x, left_y)
            self.right_joystick_widget.update_position(right_x, right_y)
            
            # Control selected instrument with joysticks
            self.handle_joystick_control(left_x, left_y, right_x, right_y)
            
            # Update trigger values
            left_trigger = (self.controller.get_axis(4) + 1) * 50  # Convert -1,1 to 0,100
            right_trigger = (self.controller.get_axis(5) + 1) * 50
            
            self.lt_progress.setValue(int(left_trigger))
            self.rt_progress.setValue(int(right_trigger))
            
            # Check button presses
            self.check_button_presses()
            
        except Exception as e:
            print(f"Error reading controller input: {e}")
            self.disconnect_controller()
            
    def check_button_presses(self):
        """Check for button presses and handle them"""
        if not self.controller:
            return
            
        # Button mapping for Xbox controller
        button_map = {
            0: 'A',
            1: 'B', 
            2: 'X',
            3: 'Y',
            4: 'LB',
            5: 'RB',
            6: 'Back',
            7: 'Start',
            8: 'Left_Stick',
            9: 'Right_Stick'
        }
        
        # Track if any button is pressed
        any_button_pressed = False
        
        # Check each button
        for button_id, button_name in button_map.items():
            if self.controller.get_button(button_id):
                self.handle_button_press(button_name)
                any_button_pressed = True
                break  # Only handle one button at a time to avoid conflicts
        
        # If no buttons are pressed, reset all button highlights
        if not any_button_pressed:
            self.reset_action_buttons()
            self.reset_bumper_buttons()
                
        # Check D-pad (hat)
        if self.controller.get_numhats() > 0:
            hat = self.controller.get_hat(0)
            if hat != (0, 0):
                self.handle_dpad_press(hat)
            else:
                # Reset D-pad buttons when no direction is pressed
                self.reset_dpad_buttons()
                
    def handle_button_press(self, button_name):
        """Handle specific button press actions"""
        print(f"Button pressed: {button_name}")
        
        # Reset all button highlights first
        self.reset_action_buttons()
        self.reset_bumper_buttons()
        
        # Add your custom button handling here
        if button_name == 'A':
            # Select Instrument 1
            self.select_instrument(1)
            print("A button pressed - Instrument 1 selected")
            self.highlight_action_button(self.btn_A)
            
        elif button_name == 'B':
            # Select Instrument 2
            self.select_instrument(2)
            print("B button pressed - Instrument 2 selected")
            self.highlight_action_button(self.btn_B)
            
        elif button_name == 'X':
            # Select Instrument 3
            self.select_instrument(3)
            print("X button pressed - Instrument 3 selected")
            self.highlight_action_button(self.btn_X)
            
        elif button_name == 'Y':
            # Deselect all instruments
            self.select_instrument(None)
            print("Y button pressed - All instruments deselected")
            self.highlight_action_button(self.btn_Y)
            
        elif button_name == 'LB':
            print("Left Bumper pressed")
            self.highlight_bumper_button(self.lb_button)
            
        elif button_name == 'RB':
            print("Right Bumper pressed")
            self.highlight_bumper_button(self.rb_button)
            
        elif button_name == 'Start':
            print("Start button pressed")
            
        elif button_name == 'Back':
            print("Back button pressed")
    
    def select_instrument(self, instrument_num):
        """Select an instrument for control"""
        # Reset all instruments
        for i in range(1, 4):
            self.instrument_status[i] = False
            self.update_instrument_status(i, False)
        
        # Select the specified instrument
        if instrument_num is not None:
            self.selected_instrument = instrument_num
            self.instrument_status[instrument_num] = True
            self.update_instrument_status(instrument_num, True)
            print(f"Instrument {instrument_num} selected for control")
        else:
            self.selected_instrument = None
            print("No instrument selected")
    
    def handle_joystick_control(self, left_x, left_y, right_x, right_y):
        """Handle joystick control for selected instrument"""
        if self.selected_instrument is None or self.main_window is None:
            return
        
        # Apply deadzone
        if abs(left_y) < self.joystick_deadzone:
            left_y = 0
        if abs(right_x) < self.joystick_deadzone:
            right_x = 0
        
        # Only process if there's significant movement
        if abs(left_y) > 0 or abs(right_x) > 0:
            # Scale joystick input
            translation_input = left_y * self.joystick_sensitivity
            rotation_input = right_x * self.joystick_sensitivity
            
            # Control the selected instrument
            if self.selected_instrument == 1:
                self.control_instrument_1(translation_input, rotation_input)
            elif self.selected_instrument == 2:
                self.control_instrument_2(translation_input, rotation_input)
            elif self.selected_instrument == 3:
                self.control_instrument_3(translation_input, rotation_input)
    
    def control_instrument_1(self, translation_input, rotation_input):
        """Control Instrument 1 with joystick input"""
        if translation_input != 0:
            # Use left joystick Y-axis for translation (insertion/withdrawal) 
            # This replicates the exact behavior of I1_T_UpBtn_Clicked and I1_T_DownBtn_Clicked
            self.main_window.Rotation_Button_Clicked = False
            
            if translation_input > 0:  # Forward on joystick = withdrawal (like pressing I1_T_upbtn)
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I1_T_Effector_posref = self.main_window.Compute_Increment(
                        self.main_window.I1_T_Effector_posref, self.main_window.I1_T_sclebox)
                    self.main_window.I1_T_Motor_posref = self.main_window.Translation_Transform_Catheter_to_M2(
                        self.main_window.I1_T_Effector_posref)
                else:
                    self.main_window.I1_T_Motor_posref = self.main_window.Compute_Increment(
                        self.main_window.I1_T_Motor_posref, self.main_window.I1_T_sclebox)
                        
            else:  # Backward on joystick = insertion (like pressing I1_T_downbtn)
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I1_T_Effector_posref = self.main_window.Compute_Decrement(
                        self.main_window.I1_T_Effector_posref, self.main_window.I1_T_sclebox)
                    self.main_window.I1_T_Motor_posref = self.main_window.Translation_Transform_Catheter_to_M2(
                        self.main_window.I1_T_Effector_posref)
                else:
                    self.main_window.I1_T_Motor_posref = self.main_window.Compute_Decrement(
                        self.main_window.I1_T_Motor_posref, self.main_window.I1_T_sclebox)
            
            self.main_window.Transmit_data()
        
        if rotation_input != 0:
            # Use right joystick X-axis for rotation
            # This replicates the exact behavior of I1_R_UpBtn_Clicked and I1_R_DownBtn_Clicked
            self.main_window.Rotation_Button_Clicked = True
            
            if rotation_input > 0:  # Right on joystick = CW rotation (like pressing I1_R_upbtn)
                self.main_window.CW_Rotation = True
                self.main_window.CCW_Rotation = False
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I1_R_Effector_posref = self.main_window.Compute_Increment(
                        self.main_window.I1_R_Effector_posref, self.main_window.I1_R_sclebox)
                    self.main_window.I1_R_Motor_posref = self.main_window.Rotation_Transform_Catheter_to_M1(
                        self.main_window.I1_R_Effector_posref)
                else:
                    self.main_window.I1_R_Motor_posref = self.main_window.Compute_Increment(
                        self.main_window.I1_R_Motor_posref, self.main_window.I1_R_sclebox)
                        
            else:  # Left on joystick = CCW rotation (like pressing I1_R_downbtn)
                self.main_window.CW_Rotation = False
                self.main_window.CCW_Rotation = True
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I1_R_Effector_posref = self.main_window.Compute_Decrement(
                        self.main_window.I1_R_Effector_posref, self.main_window.I1_R_sclebox)
                    self.main_window.I1_R_Motor_posref = self.main_window.Rotation_Transform_Catheter_to_M1(
                        self.main_window.I1_R_Effector_posref)
                else:
                    self.main_window.I1_R_Motor_posref = self.main_window.Compute_Decrement(
                        self.main_window.I1_R_Motor_posref, self.main_window.I1_R_sclebox)
            
            # Compensate coupling (this is also done in the button methods)
            self.main_window.I1_T_Motor_posref = self.main_window.Decouple_Rotation_Translation_M1_to_M2(
                self.main_window.I1_R_Motor_posref, self.main_window.I1_T_Motor_posref, self.main_window.I1_R_sclebox)
            
            self.main_window.Transmit_data()
    
    def control_instrument_2(self, translation_input, rotation_input):
        """Control Instrument 2 with joystick input"""
        if translation_input != 0:
            # Use left joystick Y-axis for translation (insertion/withdrawal)
            # This replicates the exact behavior of I2_T_UpBtn_Clicked and I2_T_DownBtn_Clicked
            self.main_window.Rotation_Button_Clicked = False
            
            if translation_input > 0:  # Forward on joystick = withdrawal (like pressing I2_T_upbtn)
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I2_T_Effector_posref = self.main_window.Compute_Increment(
                        self.main_window.I2_T_Effector_posref, self.main_window.I2_T_sclebox)
                    self.main_window.I2_T_Motor_posref = self.main_window.Translation_Transform_Catheter_to_M2(
                        self.main_window.I2_T_Effector_posref)
                else:
                    self.main_window.I2_T_Motor_posref = self.main_window.Compute_Increment(
                        self.main_window.I2_T_Motor_posref, self.main_window.I2_T_sclebox)
                        
            else:  # Backward on joystick = insertion (like pressing I2_T_downbtn)
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I2_T_Effector_posref = self.main_window.Compute_Decrement(
                        self.main_window.I2_T_Effector_posref, self.main_window.I2_T_sclebox)
                    self.main_window.I2_T_Motor_posref = self.main_window.Translation_Transform_Catheter_to_M2(
                        self.main_window.I2_T_Effector_posref)
                else:
                    self.main_window.I2_T_Motor_posref = self.main_window.Compute_Decrement(
                        self.main_window.I2_T_Motor_posref, self.main_window.I2_T_sclebox)
            
            self.main_window.Transmit_data()
        
        if rotation_input != 0:
            # Use right joystick X-axis for rotation
            # This replicates the exact behavior of I2_R_UpBtn_Clicked and I2_R_DownBtn_Clicked
            self.main_window.Rotation_Button_Clicked = True
            
            if rotation_input > 0:  # Right on joystick = CW rotation (like pressing I2_R_upbtn)
                self.main_window.CW_Rotation = True
                self.main_window.CCW_Rotation = False
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I2_R_Effector_posref = self.main_window.Compute_Increment(
                        self.main_window.I2_R_Effector_posref, self.main_window.I2_R_sclebox)
                    self.main_window.I2_R_Motor_posref = self.main_window.Rotation_Transform_Catheter_to_M1(
                        self.main_window.I2_R_Effector_posref)
                else:
                    self.main_window.I2_R_Motor_posref = self.main_window.Compute_Increment(
                        self.main_window.I2_R_Motor_posref, self.main_window.I2_R_sclebox)
                        
            else:  # Left on joystick = CCW rotation (like pressing I2_R_downbtn)
                self.main_window.CW_Rotation = False
                self.main_window.CCW_Rotation = True
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I2_R_Effector_posref = self.main_window.Compute_Decrement(
                        self.main_window.I2_R_Effector_posref, self.main_window.I2_R_sclebox)
                    self.main_window.I2_R_Motor_posref = self.main_window.Rotation_Transform_Catheter_to_M1(
                        self.main_window.I2_R_Effector_posref)
                else:
                    self.main_window.I2_R_Motor_posref = self.main_window.Compute_Decrement(
                        self.main_window.I2_R_Motor_posref, self.main_window.I2_R_sclebox)
            
            # Compensate coupling (this is also done in the button methods)
            self.main_window.I2_T_Motor_posref = self.main_window.Decouple_Rotation_Translation_M1_to_M2(
                self.main_window.I2_R_Motor_posref, self.main_window.I2_T_Motor_posref, self.main_window.I2_R_sclebox)
            
            self.main_window.Transmit_data()
    
    def control_instrument_3(self, translation_input, rotation_input):
        """Control Instrument 3 with joystick input"""
        if translation_input != 0:
            # Use left joystick Y-axis for translation (insertion/withdrawal)
            # This replicates the exact behavior of I3_T_UpBtn_Clicked and I3_T_DownBtn_Clicked
            self.main_window.Rotation_Button_Clicked = False
            
            if translation_input > 0:  # Forward on joystick = withdrawal (like pressing I3_T_upbtn)
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I3_T_Effector_posref = self.main_window.Compute_Increment(
                        self.main_window.I3_T_Effector_posref, self.main_window.I3_T_sclebox)
                    self.main_window.I3_T_Motor_posref = self.main_window.Translation_Transform_Catheter_to_M2(
                        self.main_window.I3_T_Effector_posref)
                else:
                    self.main_window.I3_T_Motor_posref = self.main_window.Compute_Increment(
                        self.main_window.I3_T_Motor_posref, self.main_window.I3_T_sclebox)
                        
            else:  # Backward on joystick = insertion (like pressing I3_T_downbtn)
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I3_T_Effector_posref = self.main_window.Compute_Decrement(
                        self.main_window.I3_T_Effector_posref, self.main_window.I3_T_sclebox)
                    self.main_window.I3_T_Motor_posref = self.main_window.Translation_Transform_Catheter_to_M2(
                        self.main_window.I3_T_Effector_posref)
                else:
                    self.main_window.I3_T_Motor_posref = self.main_window.Compute_Decrement(
                        self.main_window.I3_T_Motor_posref, self.main_window.I3_T_sclebox)
            
            self.main_window.Transmit_data()
        
        if rotation_input != 0:
            # Use right joystick X-axis for rotation
            # This replicates the exact behavior of I3_R_UpBtn_Clicked and I3_R_DownBtn_Clicked
            self.main_window.Rotation_Button_Clicked = True
            
            if rotation_input > 0:  # Right on joystick = CW rotation (like pressing I3_R_upbtn)
                self.main_window.CW_Rotation = True
                self.main_window.CCW_Rotation = False
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I3_R_Effector_posref = self.main_window.Compute_Increment(
                        self.main_window.I3_R_Effector_posref, self.main_window.I3_R_sclebox)
                    self.main_window.I3_R_Motor_posref = self.main_window.Rotation_Transform_Catheter_to_M1(
                        self.main_window.I3_R_Effector_posref)
                else:
                    self.main_window.I3_R_Motor_posref = self.main_window.Compute_Increment(
                        self.main_window.I3_R_Motor_posref, self.main_window.I3_R_sclebox)
                        
            else:  # Left on joystick = CCW rotation (like pressing I3_R_downbtn)
                self.main_window.CW_Rotation = False
                self.main_window.CCW_Rotation = True
                if self.main_window.Control_Mode_Flag == 5:
                    self.main_window.I3_R_Effector_posref = self.main_window.Compute_Decrement(
                        self.main_window.I3_R_Effector_posref, self.main_window.I3_R_sclebox)
                    self.main_window.I3_R_Motor_posref = self.main_window.Rotation_Transform_Catheter_to_M1(
                        self.main_window.I3_R_Effector_posref)
                else:
                    self.main_window.I3_R_Motor_posref = self.main_window.Compute_Decrement(
                        self.main_window.I3_R_Motor_posref, self.main_window.I3_R_sclebox)
            
            # Compensate coupling (this is also done in the button methods)
            self.main_window.I3_T_Motor_posref = self.main_window.Decouple_Rotation_Translation_M1_to_M2(
                self.main_window.I3_R_Motor_posref, self.main_window.I3_T_Motor_posref, self.main_window.I3_R_sclebox)
            
            self.main_window.Transmit_data()
    
    def reset_action_buttons(self):
        """Reset all action buttons to normal style"""
        # Action button styles
        button_styles = {
            'Y': "background-color: #FFA500; color: white;",  # Orange
            'X': "background-color: #0066CC; color: white;",  # Blue
            'A': "background-color: #00AA00; color: white;",  # Green
            'B': "background-color: #CC0000; color: white;"   # Red
        }
        
        base_style = """
            QPushButton {
                border: 4px solid #555;
                font: bold 22px;
                min-width: 45px;
                min-height: 45px;
                border-radius: 22px;
            }
            QPushButton:pressed {
                border: 6px solid white;
            }
        """
        
        self.btn_Y.setStyleSheet(base_style + button_styles['Y'])
        self.btn_X.setStyleSheet(base_style + button_styles['X'])
        self.btn_A.setStyleSheet(base_style + button_styles['A'])
        self.btn_B.setStyleSheet(base_style + button_styles['B'])
    
    def highlight_action_button(self, button):
        """Highlight a specific action button when pressed"""
        # Get the button's original color
        button_colors = {
            self.btn_Y: "#FFA500",  # Orange
            self.btn_X: "#0066CC",  # Blue
            self.btn_A: "#00AA00",  # Green
            self.btn_B: "#CC0000"   # Red
        }
        
        original_color = button_colors.get(button, "#FFA500")
        
        highlighted_style = f"""
            QPushButton {{
                background-color: {original_color};
                border: 6px solid #ffff00;
                color: yellow;
                font: bold 22px;
                min-width: 45px;
                min-height: 45px;
                border-radius: 22px;
            }}
        """
        button.setStyleSheet(highlighted_style)
    
    def reset_bumper_buttons(self):
        """Reset bumper buttons to normal style"""
        normal_style = """
            QPushButton {
                background-color: #333;
                border: 4px solid #555;
                color: white;
                font: bold 22px;
                min-height: 25px;
                border-radius: 5px;
            }
            QPushButton:pressed {
                border: 6px solid white;
            }
        """
        
        self.lb_button.setStyleSheet(normal_style)
        self.rb_button.setStyleSheet(normal_style)
    
    def highlight_bumper_button(self, button):
        """Highlight a specific bumper button when pressed"""
        highlighted_style = """
            QPushButton {
                background-color: #555;
                border: 6px solid #ffff00;
                color: yellow;
                font: bold 22px;
                min-height: 25px;
                border-radius: 5px;
            }
        """
        button.setStyleSheet(highlighted_style)
            
    def handle_dpad_press(self, hat_value):
        """Handle D-pad press actions"""
        x, y = hat_value
        
        # Reset all D-pad button styles first
        self.reset_dpad_buttons()
        
        if x == -1:
            print("D-pad Left pressed")
            self.highlight_dpad_button(self.dpad_left)
        elif x == 1:
            print("D-pad Right pressed")
            self.highlight_dpad_button(self.dpad_right)
            
        if y == 1:
            print("D-pad Up pressed")
            self.highlight_dpad_button(self.dpad_up)
        elif y == -1:
            print("D-pad Down pressed")
            self.highlight_dpad_button(self.dpad_down)
    
    def reset_dpad_buttons(self):
        """Reset all D-pad buttons to normal style"""
        normal_style = """
            QPushButton {
                background-color: #333;
                border: 4px solid #555;
                color: white;
                font: bold 22px;
                min-width: 40px;
                min-height: 40px;
            }
            QPushButton:pressed {
                border: 6px solid white;
            }
        """
        
        self.dpad_up.setStyleSheet(normal_style)
        self.dpad_down.setStyleSheet(normal_style)
        self.dpad_left.setStyleSheet(normal_style)
        self.dpad_right.setStyleSheet(normal_style)
    
    def highlight_dpad_button(self, button):
        """Highlight a specific D-pad button when pressed"""
        highlighted_style = """
            QPushButton {
                background-color: #555;
                border: 6px solid #ffff00;
                color: yellow;
                font: bold 22px;
                min-width: 40px;
                min-height: 40px;
            }
        """
        button.setStyleSheet(highlighted_style)

    def create_xbox_controller_layout(self):
        """Create an Xbox controller-like interface layout"""
        
        # Main controller container
        controller_frame = QtWidgets.QFrame()
        controller_frame.setStyleSheet(""" 
            QFrame {
                background-color: #2C2C2C;
                border: 4px solid #555;
                border-radius: 20px;
                padding: 28px;
            }
        """)
        controller_frame.setFixedSize(1600, 1200)
        
        # Create main grid layout for controller
        main_layout = QtWidgets.QGridLayout(controller_frame)
        main_layout.setSpacing(20)
        
        # === LEFT SIDE ===
        # Left Joystick
        left_joystick_group = self.create_joystick_group("Left Joystick", "left")
        main_layout.addWidget(left_joystick_group, 2, 0, 1, 2)
        
        # D-Pad
        dpad_group = self.create_dpad_group()
        main_layout.addWidget(dpad_group, 1, 0)
        
        # === CENTER ===
        # Camera Video Feed
        camera_group = self.create_camera_group()
        main_layout.addWidget(camera_group, 1, 1, 1, 3)
        
        # === RIGHT SIDE ===
        # Right Joystick  
        right_joystick_group = self.create_joystick_group("Right Joystick", "right")
        main_layout.addWidget(right_joystick_group, 2, 3, 1, 2)
        
        # Action Buttons (A, B, X, Y)
        action_buttons_group = self.create_action_buttons_group()
        main_layout.addWidget(action_buttons_group, 1, 4)
        
        # === TRIGGERS AND BUMPERS ===
        # Top triggers and bumpers
        triggers_group = self.create_triggers_group()
        main_layout.addWidget(triggers_group, 0, 0, 1, 5)
        
        # Add controller frame to main layout
        self.layout.addWidget(controller_frame, alignment=Qt.AlignCenter)
        
        # Status display
        self.create_status_display()
        
    def create_joystick_group(self, title, side):
        """Create a joystick control group"""
        group = QtWidgets.QGroupBox(title)
        group.setStyleSheet("""
            QGroupBox {
                font: bold 22px;
                color: white;
                border: 2px solid #555;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        
        layout = QtWidgets.QVBoxLayout(group)
        
        # Create custom joystick widget with visual feedback
        joystick_widget = JoystickWidget(side)
        joystick_widget.setFixedSize(120, 120)
        
        # Store reference for later updates
        setattr(self, f"{side}_joystick_widget", joystick_widget)
        
        # Joystick values display
        x_label = QtWidgets.QLabel(f"{side.capitalize()} X: 0.00")
        y_label = QtWidgets.QLabel(f"{side.capitalize()} Y: 0.00")
        x_label.setStyleSheet(f"font-size: {self.overall_fontsize}px; color: white;")
        y_label.setStyleSheet(f"font-size: {self.overall_fontsize}px; color: white;")

        # Store references for later updates
        setattr(self, f"{side}_x_label", x_label)
        setattr(self, f"{side}_y_label", y_label)
        
        layout.addWidget(joystick_widget, alignment=Qt.AlignCenter)
        layout.addWidget(x_label)
        layout.addWidget(y_label)
        
        return group
    
    def create_dpad_group(self):
        """Create D-Pad control group"""
        group = QtWidgets.QGroupBox("D-Pad")
        group.setStyleSheet("""
            QGroupBox {
                font: bold 22px;
                color: white;
                border: 4px solid #555;
                border-radius: 8px;
                margin-top: 10px;
            }
        """)
        
        layout = QtWidgets.QGridLayout(group)
        
        # D-Pad buttons
        dpad_style = """
            QPushButton {
                background-color: #333;
                border: 4px solid #555;
                color: white;
                font: bold 22px;
                min-width: 40px;
                min-height: 40px;
            }
            QPushButton:pressed {
                border: 6px solid white;
            }
        """
        
        self.dpad_up = QtWidgets.QPushButton("↑")
        self.dpad_down = QtWidgets.QPushButton("↓")
        self.dpad_left = QtWidgets.QPushButton("←")
        self.dpad_right = QtWidgets.QPushButton("→")
        
        for btn in [self.dpad_up, self.dpad_down, self.dpad_left, self.dpad_right]:
            btn.setStyleSheet(dpad_style)
        
        layout.addWidget(self.dpad_up, 0, 1)
        layout.addWidget(self.dpad_left, 1, 0)
        layout.addWidget(self.dpad_right, 1, 2)
        layout.addWidget(self.dpad_down, 2, 1)
        
        return group
    
    def create_action_buttons_group(self):
        """Create action buttons group (A, B, X, Y)"""
        group = QtWidgets.QGroupBox("Action Buttons")
        group.setStyleSheet("""
            QGroupBox {
                font: bold 22px;
                color: white;
                border: 4px solid #555;
                border-radius: 8px;
                margin-top: 10px;
            }
        """)
        
        layout = QtWidgets.QGridLayout(group)
        
        # Action button styles
        button_styles = {
            'Y': "background-color: #FFA500; color: white;",  # Orange
            'X': "background-color: #0066CC; color: white;",  # Blue
            'A': "background-color: #00AA00; color: white;",  # Green
            'B': "background-color: #CC0000; color: white;"   # Red
        }
        
        base_style = """
            QPushButton {
                border: 4px solid #555;
                font: bold 22px;
                min-width: 45px;
                min-height: 45px;
                border-radius: 22px;
            }
            QPushButton:pressed {
                border: 6px solid white;
            }
        """
        
        self.btn_Y = QtWidgets.QPushButton("Y")
        self.btn_X = QtWidgets.QPushButton("X")
        self.btn_A = QtWidgets.QPushButton("A")
        self.btn_B = QtWidgets.QPushButton("B")
        
        buttons = {'Y': self.btn_Y, 'X': self.btn_X, 'A': self.btn_A, 'B': self.btn_B}
        
        for name, btn in buttons.items():
            btn.setStyleSheet(base_style + button_styles[name])
        
        layout.addWidget(self.btn_Y, 0, 1)
        layout.addWidget(self.btn_X, 1, 0)
        layout.addWidget(self.btn_B, 1, 2)
        layout.addWidget(self.btn_A, 2, 1)
        
        return group
    
    def create_triggers_group(self):
        """Create triggers and bumpers group"""
        group = QtWidgets.QGroupBox("Triggers & Bumpers")
        group.setStyleSheet("""
            QGroupBox {
                font: bold 22px;
                color: white;
                border: 4px solid #555;
                border-radius: 8px;
                margin-top: 10px;
            }
        """)
        
        layout = QtWidgets.QHBoxLayout(group)
        
        # Left side
        left_layout = QtWidgets.QVBoxLayout()
        self.lb_button = QtWidgets.QPushButton("LB")
        self.lt_progress = QtWidgets.QProgressBar()
        self.lt_progress.setMaximum(100)
        self.lt_progress.setTextVisible(True)
        self.lt_progress.setFormat("LT: %v%")
        
        left_layout.addWidget(self.lb_button)
        left_layout.addWidget(self.lt_progress)
        
        # Right side
        right_layout = QtWidgets.QVBoxLayout()
        self.rb_button = QtWidgets.QPushButton("RB")
        self.rt_progress = QtWidgets.QProgressBar()
        self.rt_progress.setMaximum(100)
        self.rt_progress.setTextVisible(True)
        self.rt_progress.setFormat("RT: %v%")
        
        right_layout.addWidget(self.rb_button)
        right_layout.addWidget(self.rt_progress)
        
        layout.addLayout(left_layout)
        layout.addStretch()
        layout.addLayout(right_layout)
        
        # Style buttons and progress bars
        button_style = """
            QPushButton {
                background-color: #333;
                border: 4px solid #555;
                color: white;
                font: bold 22px;
                min-height: 25px;
                border-radius: 5px;
            }
            QPushButton:pressed {
                border: 6px solid white;
            }
        """
        
        self.lb_button.setStyleSheet(button_style)
        self.rb_button.setStyleSheet(button_style)
        
        return group
    
    def create_camera_group(self):
        """Create camera video feed group"""
        group = QtWidgets.QGroupBox("Real-Time Camera Feed")
        group.setStyleSheet("""
            QGroupBox {
                font: bold 24px;
                color: white;
                border: 4px solid #555;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        
        layout = QtWidgets.QVBoxLayout(group)
        
        # Camera display widget (placeholder for now)
        self.camera_display = QtWidgets.QLabel()
        self.camera_display.setFixedSize(600, 300)
        self.camera_display.setStyleSheet("""
            QLabel {
                background-color: #1a1a1a;
                border: 2px solid #666;
                border-radius: 5px;
                color: #888;
                font-size: 18px;
                font-weight: bold;
            }
        """)
        self.camera_display.setText("Camera Feed\n(Not Connected)")
        self.camera_display.setAlignment(Qt.AlignCenter)
        self.camera_display.setScaledContents(True)
        
        # Camera control buttons
        camera_controls_layout = QtWidgets.QHBoxLayout()
        
        self.camera_connect_btn = QtWidgets.QPushButton("Connect Camera")
        self.camera_connect_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: #4CAF50;
                color: white;
                font: bold {self.overall_fontsize-4}px;
                padding: 8px;
                border: none;
                border-radius: 5px;
                min-width: 120px;
            }}
            QPushButton:hover {{
                background-color: #45a049;
            }}
            QPushButton:pressed {{
                background-color: #3d8b40;
            }}
        """)
        self.camera_connect_btn.clicked.connect(self.toggle_camera_connection)
        
        self.camera_record_btn = QtWidgets.QPushButton("Record")
        self.camera_record_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: #f44336;
                color: white;
                font: bold {self.overall_fontsize-4}px;
                padding: 8px;
                border: none;
                border-radius: 5px;
                min-width: 80px;
            }}
            QPushButton:hover {{
                background-color: #da190b;
            }}
        """)
        self.camera_record_btn.clicked.connect(self.toggle_camera_recording)
        self.camera_record_btn.setEnabled(False)  # Disabled until camera connects
        
        self.camera_snapshot_btn = QtWidgets.QPushButton("Snapshot")
        self.camera_snapshot_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: #2196F3;
                color: white;
                font: bold {self.overall_fontsize-4}px;
                padding: 8px;
                border: none;
                border-radius: 5px;
                min-width: 80px;
            }}
            QPushButton:hover {{
                background-color: #1976D2;
            }}
        """)
        self.camera_snapshot_btn.clicked.connect(self.take_camera_snapshot)
        self.camera_snapshot_btn.setEnabled(False)  # Disabled until camera connects
        
        camera_controls_layout.addWidget(self.camera_connect_btn)
        camera_controls_layout.addWidget(self.camera_record_btn)
        camera_controls_layout.addWidget(self.camera_snapshot_btn)
        camera_controls_layout.addStretch()
        
        # Camera status
        self.camera_status_label = QtWidgets.QLabel("Camera Status: Disconnected")
        self.camera_status_label.setStyleSheet("color: #ff4444; font: bold 16px;")
        
        # Add widgets to layout
        layout.addWidget(self.camera_display, alignment=Qt.AlignCenter)
        layout.addLayout(camera_controls_layout)
        layout.addWidget(self.camera_status_label)
        
        # Camera state variables
        self.camera_connected = False
        self.camera_recording = False
        
        return group
    
    def toggle_camera_connection(self):
        """Toggle camera connection (placeholder for actual implementation)"""
        if not self.camera_connected:
            # Simulate camera connection
            self.camera_connected = True
            self.camera_connect_btn.setText("Disconnect Camera")
            self.camera_connect_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: #f44336;
                    color: white;
                    font: bold {self.overall_fontsize-4}px;
                    padding: 8px;
                    border: none;
                    border-radius: 5px;
                    min-width: 120px;
                }}
                QPushButton:hover {{
                    background-color: #da190b;
                }}
            """)
            
            self.camera_display.setStyleSheet("""
                QLabel {
                    background-color: #0a5a0a;
                    border: 2px solid #4CAF50;
                    border-radius: 5px;
                    color: #4CAF50;
                    font-size: 18px;
                    font-weight: bold;
                }
            """)
            self.camera_display.setText("Camera Feed\n(Simulated - Ready)")
            
            self.camera_status_label.setText("Camera Status: Connected")
            self.camera_status_label.setStyleSheet("color: #44ff44; font: bold 16px;")
            
            # Enable camera controls
            self.camera_record_btn.setEnabled(True)
            self.camera_snapshot_btn.setEnabled(True)
            
            print("Camera connected (simulated)")
            
        else:
            # Disconnect camera
            self.camera_connected = False
            self.camera_recording = False
            
            self.camera_connect_btn.setText("Connect Camera")
            self.camera_connect_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: #4CAF50;
                    color: white;
                    font: bold {self.overall_fontsize-4}px;
                    padding: 8px;
                    border: none;
                    border-radius: 5px;
                    min-width: 120px;
                }}
                QPushButton:hover {{
                    background-color: #45a049;
                }}
            """)
            
            self.camera_display.setStyleSheet("""
                QLabel {
                    background-color: #1a1a1a;
                    border: 2px solid #666;
                    border-radius: 5px;
                    color: #888;
                    font-size: 18px;
                    font-weight: bold;
                }
            """)
            self.camera_display.setText("Camera Feed\n(Not Connected)")
            
            self.camera_status_label.setText("Camera Status: Disconnected")
            self.camera_status_label.setStyleSheet("color: #ff4444; font: bold 16px;")
            
            # Disable camera controls
            self.camera_record_btn.setEnabled(False)
            self.camera_snapshot_btn.setEnabled(False)
            self.camera_record_btn.setText("Record")
            
            print("Camera disconnected")
    
    def toggle_camera_recording(self):
        """Toggle camera recording (placeholder for actual implementation)"""
        if not self.camera_recording:
            self.camera_recording = True
            self.camera_record_btn.setText("Stop Recording")
            self.camera_record_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: #ff6b6b;
                    color: white;
                    font: bold {self.overall_fontsize-4}px;
                    padding: 8px;
                    border: none;
                    border-radius: 5px;
                    min-width: 80px;
                }}
                QPushButton:hover {{
                    background-color: #ff5252;
                }}
            """)
            
            self.camera_display.setText("Camera Feed\n(Recording...)")
            print("Camera recording started")
            
        else:
            self.camera_recording = False
            self.camera_record_btn.setText("Record")
            self.camera_record_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: #f44336;
                    color: white;
                    font: bold {self.overall_fontsize-4}px;
                    padding: 8px;
                    border: none;
                    border-radius: 5px;
                    min-width: 80px;
                }}
                QPushButton:hover {{
                    background-color: #da190b;
                }}
            """)
            
            self.camera_display.setText("Camera Feed\n(Simulated - Ready)")
            print("Camera recording stopped")
    
    def take_camera_snapshot(self):
        """Take a camera snapshot (placeholder for actual implementation)"""
        if self.camera_connected:
            print("Camera snapshot taken")
            # Here you would implement actual snapshot functionality
            # For now, just provide visual feedback
            original_text = self.camera_display.text()
            self.camera_display.setText("Snapshot Taken!")
            
            # Reset text after a short delay
            QtCore.QTimer.singleShot(1000, lambda: self.camera_display.setText(original_text))
        else:
            print("Cannot take snapshot: camera not connected")
    
    def create_status_display(self):
        """Create status display area"""
        status_group = QtWidgets.QGroupBox("Controller Status")
        status_group.setStyleSheet("""
            QGroupBox {
                font: bold 22px;
                color: white;
                border: 4px solid #555;
                border-radius: 8px;
                margin-top: 10px;
            }
        """)
        
        layout = QtWidgets.QHBoxLayout(status_group)

        self.connectxobx_button = QtWidgets.QPushButton("Connect Xbox")
        
        self.connection_status = QtWidgets.QLabel("Status: Disconnected")
        self.connection_status.setStyleSheet("color: #ff4444; font: bold 22px;")

        self.battery_level = QtWidgets.QLabel("Battery: N/A")
        self.battery_level.setStyleSheet("color: #44ff44; font: bold 22px;")
        
        layout.addWidget(self.connection_status)
        layout.addStretch()
        layout.addWidget(self.battery_level)
        
        self.layout.addWidget(status_group)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    Window = MainWindow()
    Window.show()
    sys.exit(app.exec_())