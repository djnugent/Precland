import ctypes
from ctypes import *
import numpy as np
import time

solocam = CDLL('/home/root/precland/Precland/libsolocam.so')

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

class SoloCamera(object):
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
    self.reading = False

    try:
        self._start()
        self.opened = True
    except OSError:
        self.opened = False

  def _start(self):
    check(solocam.solocam_start(self.ctx))

  def stop(self):
    check(solocam.solocam_stop(self.ctx))

  def isOpened(self):
    return self.opened and self.width == 1280 and self.height == 720

  def clear(self):
    if not self.reading:
        bufp = BUF_P()
        check(solocam.solocam_read_frame(self.ctx, byref(bufp)))
        check(solocam.solocam_free_frame(self.ctx, bufp))
    else:
        time.sleep(0.016)

  def read(self):
    self.reading = True
    bufp = BUF_P()
    check(solocam.solocam_read_frame(self.ctx, byref(bufp)))
    bufc = bufp.contents
    image = np.ctypeslib.as_array(bufc.data,shape = (self.height*self.width,)).reshape(self.height, self.width)
    cp_image = np.empty_like(image)
    np.copyto(cp_image,image)
    check(solocam.solocam_free_frame(self.ctx, bufp))
    self.reading = False
    return True, cp_image

  def __del__(self):
    check(solocam.solocam_close(self.ctx))

if __name__ == "__main__":
    import cv2
    import time
    cam = SoloCamera()
    if cam.isOpened():
        print "Capturing 10 images..."
        #for i in range(0,10):
        while True:
            start = time.time()
            ret, frame = cam.read()
            if frame is None:
                print "None image"
            stop = time.time()
            print 1.0 / (stop-start)
    else:
        print "failed to open gopro"

    print "closing camera..."
