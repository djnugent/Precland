import ctypes
from ctypes import *
import numpy as np
solocam = CDLL('./libsolocam.so')

def check(val):
  if val != 0:
    raise OSError("Error in solocam C call (see stderr)")


class BUF(Structure):
  _fields_ = [("id", c_int),
              ("data", POINTER(c_ubyte)),
              ("length", c_size_t),
              ("used", c_size_t),
              ]

BUF_P = POINTER(BUF)

class SoloCam(object):
  def __init__(self):
    self.ctx = c_void_p()
    check(solocam.solocam_open_hdmi(byref(self.ctx)))
    check(solocam.solocam_set_format_720p60_grayscale(self.ctx))

    width = c_int()
    height = c_int()
    check(solocam.solocam_get_size(self.ctx, byref(width), byref(height)))
    self.width = width.value
    self.height = height.value
    self.opened = False
    try:
        self.start()
        self.opened = True
    except IOError:
        self.opened = False

  def start(self):
    check(solocam.solocam_start(self.ctx))

  def stop(self):
    check(solocam.solocam_stop(self.ctx))

  def isOpened(self):
    return self.opened and self.width == 1280 and self.height == 720

  def read(self):
    bufp = BUF_P()
    check(solocam.solocam_read_frame(self.ctx, byref(bufp)))
    bufc = bufp.contents
    #r = bytearray(bufc.data[0:bufc.used])
    r = np.array(bufc.data[0:self.height*self.width],dtype='uint8').reshape(self.height, self.width)

    ArrayType = ctypes.c_ubyte*self.width*self.height
    addr = ctypes.addressof(bufc)
    a = np.frombuffer(ArrayType.from_address(addr))

    check(solocam.solocam_free_frame(self.ctx, bufp))

    #return True, image

  def __del__(self):
    check(solocam.solocam_close(self.ctx))

if __name__ == "__main__":
    import cv2
    import time
    cam = SoloCam()
    if cam.isOpened():
        print "Capturing 10 images..."
        while True:
            start = time.time()
            #ret, frame = cam.read()
            cam.read()
            stop = time.time()
            print (stop-start)
        #cv2.imwrite("cap.png",frame)
    else:
        print "failed to open gopro"

    print "closing camera..."
