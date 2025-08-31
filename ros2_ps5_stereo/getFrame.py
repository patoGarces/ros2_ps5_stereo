import cv2
from enum import Enum
import threading

import queue

WINDOWS_PLATFORM_NAME = 'Windows'
LINUX_PLATFORM_NAME = 'Linux'

class GetFrame:

    # out_full = cv2.VideoWriter('full.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(3448,808))
    # out_left = cv2.VideoWriter('left.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(1264,800))
    # out_right = cv2.VideoWriter('right.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(1264,800))

    def __init__(self):
        self.flagRunning = threading.Event()
        self.flagRunning.clear()
        self.hilo_emision = None
        self.queueFrames = queue.Queue(maxsize=1)
        
    def getQueueGetFrame(self):
        return self.queueFrames

    def decode(self,frame):

        height, width, _ = frame.shape
        mid = width // 2
        left_frame = frame[:, :mid, :]
        right_frame = frame[:, mid:, :]

        left_frame = cv2.resize(left_frame,(1264,800))
        right_frame = cv2.resize(right_frame,(1264,800))
        
        return (left_frame, right_frame)

    def getNewFrame(self):

        while self.flagRunning.isSet():
            ret, frame = self.cap.read()
            if not ret:
                break
            left, right = self.decode(frame)

            # self.out_full.write(frame)
            # self.out_left.write(left)
            # self.out_right.write(right)

            self.queueFrames.put((left,right))

    def startStream(self, cameraIndex, resolution, fps):
        
        self.cap = cv2.VideoCapture(cameraIndex)

        if not self.cap.isOpened:
            print("Camera is not found")
            return False
        else:
            # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
            # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)
            # self.cap.set(cv2.CAP_PROP_FPS, 8)
            try:
                if (resolution == Resolutions.RES_640p):
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)        
                elif (resolution == Resolutions.RES_1080p):   # DEBERIA SER 1280
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
                elif (resolution == Resolutions.RES_1920p):  # DEBERIA SER 
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)
                else:
                    print("Error al seleccionar la resolucion,resValue: ",resolution)

                if (fps == Fps.FPS_8):
                    self.cap.set(cv2.CAP_PROP_FPS, 8) 
                elif (fps == Fps.FPS_30):
                    self.cap.set(cv2.CAP_PROP_FPS, 30)
                elif (fps == Fps.FPS_60):
                    self.cap.set(cv2.CAP_PROP_FPS, 60)       
                else:
                    self.cap.set(cv2.CAP_PROP_FPS, 8)
                    print("Error al seleccionar los fps,resValue: ",fps)

                self.flagRunning.set()
                self.hilo_emision = threading.Thread(target=self.getNewFrame)
                self.hilo_emision.start()
                return True
            except Exception as error:
                print("error: ",error)
                return False

    def stopStream(self):
        self.flagRunning.clear()
        self.hilo_emision = None
        # self.cap.release()

    def showPreviewRgb(self,frameLeft,frameRight):
        left = cv2.resize(frameLeft,(int(256*3),int(108*4.5)))
        right = cv2.resize(frameRight,(int(256*3),int(108*4.5)))
        cv2.imshow('RGB left', left)
        cv2.imshow('RGB right', right)


class Resolutions(Enum):
    RES_1920p = 0        # res: RES_3448x808
    RES_1080p = 1        # res: RES_2560x800
    RES_640p = 2         # res: RES_640x480

class Fps(Enum):
    FPS_8 = 0
    FPS_30 = 1
    FPS_60 = 2