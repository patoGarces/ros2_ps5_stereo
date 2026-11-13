import cv2
import threading
from ros2_ps5_stereo.utilsClass import Resolutions, Utils

WINDOWS_PLATFORM_NAME = 'Windows'
LINUX_PLATFORM_NAME = 'Linux'

class GetFrame:

    # out_full = cv2.VideoWriter('full.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(3448,808))
    # out_left = cv2.VideoWriter('left.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(1264,800))
    # out_right = cv2.VideoWriter('right.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(1264,800))

    def __init__(self, logger, resolution: Resolutions, roi_height):
        self.logger = logger
        self.resolution = resolution
        self.roiHeight = roi_height
        self.flagRunning = threading.Event()
        self.flagRunning.clear()
        self.hilo_emision = None
        self.cbFrames = []
        cv2.ocl.setUseOpenCL(True)
    
    def setCbFrames(self, cb):
        self.cbFrames = cb

    def decode(self, frame):
        mid = frame.shape[1] // 2

        left_frame = frame[:, mid:, :]
        right_frame = frame[:, :mid, :]

        if self.roiHeight is not None:
            start_y = (left_frame.shape[0] - self.roiHeight) // 2
            end_y = start_y + self.roiHeight

            left_frame = left_frame[start_y:end_y, :, :]
            right_frame = right_frame[start_y:end_y, :, :]

        return left_frame, right_frame
    
    def getNewFrame(self):

        while self.flagRunning.isSet():
            ret, frame = self.cap.read()
            if not ret:
                break
            left, right = self.decode(frame)

            # self.out_full.write(frame)
            # self.out_left.write(left)
            # self.out_right.write(right)
            self.cbFrames(left, right)
        
        self.logger.error('Error getting frame')

    def startStream(self, cameraIndex, resolution):
        
        self.cap = cv2.VideoCapture(cameraIndex)
        self.resolution = resolution

        if not self.cap.isOpened():
            self.logger.error("Failed starting stream, camera not found")
            return False
        else:
            try: 
                widht , height , fps = Utils().enumToResolutionAndFps(Resolutions.RES_1920x1080_8FPS) # resolution)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, widht)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                self.cap.set(cv2.CAP_PROP_FPS, fps) 

                self.flagRunning.set()
                self.hilo_emision = threading.Thread(target=self.getNewFrame)
                self.hilo_emision.start()
                return True
            except Exception as error:
                self.logger.error("error: ",error)
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
