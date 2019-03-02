#!/usr/bin/env python

import socket
import struct
import numpy as np
from matplotlib import pyplot as plt
from threading import Thread
#import threading

import wx
from wxmplot import ImageFrame
import cv2

VELOVIEW_PORT = 2368
VELOVIEW_IP = '127.0.0.1'
HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 7080        # The port used by the server

class LidarGUISensor():
    def __init__(self, *args, **kwargs):
        self.connect()

    def forward_data(self, data):
        if self.veloview_socket is None:
            return
        if isinstance(data, list):
            print("Sending data", len(data))
            for datum in data:
                self.veloview_socket.sendto(datum, (VELOVIEW_IP, VELOVIEW_PORT))
        else:
            self.veloview_socket.sendto(data, (VELOVIEW_IP, VELOVIEW_PORT))

    def connect(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.veloview_socket = s
            return s
        except Exception as e:
            self.veloview_socket = None
            return None


class CameraFrame(wx.Frame):
    def __init__(self, *args, **kwargs):
        wx.Frame.__init__(self, *args, **kwargs)
        self.InitUI()

    def InitUI(self):
        self.bitmap = wx.Bitmap.FromRGBA(512, 512)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Centre()
        self.Show(True)

    def SetImage(self, image):
        #self.bitmap.CopyFromBuffer(image.tostring(), format=wx.BitmapBufferFormat_RGBA)
        #self.bitmap.CopyFromBuffer(image, format=wx.BitmapBufferFormat_RGBA, stride=512)
        self.bitmap = image
        print("image set")

    def OnPaint(self, e):
        print("Painting")
        dc = wx.PaintDC(self)
        brush = wx.Brush("white")
        dc.SetBackground(brush)
        dc.Clear()

        dc.DrawBitmap(self.bitmap, 0, 0, useMask=True)

app = wx.App()
frame = CameraFrame(None, title='camera')
frame.SetSize(width=512, height=512)

def read(socket, length):
    received = 0
    data = b''

    while received < length:
        recv_buffer = socket.recv(length - received)
        if recv_buffer is None or len(recv_buffer) == 0:
            raise Exception("error reading from socket")
        data += recv_buffer
        received += len(recv_buffer)

    return received, data

def runSockCAM():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Connecting to ", HOST, PORT)
        s.connect((HOST, PORT))
        header_size = 12

        while True:
            print("reading...")
            received, packet_header = read(s, header_size)
            print("header", received)
            length, time_stamp, game_time = struct.unpack('!IIf', packet_header)
            print("Length of packet", length)
            length = length - header_size
            received, packet = read(s, length)
            print("Received length", received, len(packet))
            img = np.array(bytearray(packet), dtype=np.uint8).reshape(512, 512, 4)
            cvimg = cv2.cvtColor(img, cv2.COLOR_BGRA2RGBA)
            frame.bitmap.CopyFromBuffer(cvimg, format=wx.BitmapBufferFormat_RGBA)#, stride=512)

            frame.Refresh()

def runSockLidar():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Connecting to ", HOST, 8089)
        s.connect((HOST, 8089))
        header_size = 12
        number_blocks = 360/0.4*32/32
        number_packets = int(number_blocks / 12)
        print("Number of packets", number_packets)
        lidar_buffer = []
        Lidar = LidarGUISensor()

        while True:
            received, packet_header = read(s, header_size)
            length, time_stamp, game_time = struct.unpack('!IIf', packet_header)
            print("Length of packet", length)
            length = length - header_size
            received, packet = read(s, length)
            #print("Received length", received, len(packet))
            lidar_buffer.append(packet)
            print(len(lidar_buffer), number_packets)
            #if len(lidar_buffer) == number_packets:
            #    Lidar.forward_data(lidar_buffer)
            #    lidar_buffer = []






thread1 = Thread(target = runSockCAM)
thread1.start()
#thread2 = Thread(target = runSockLidar)
#thread2.start()

app.MainLoop()

thread1.join()
#thread2.join()