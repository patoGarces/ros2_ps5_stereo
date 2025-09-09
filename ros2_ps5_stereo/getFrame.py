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

    def decode(self,frame):

        height, width, _ = frame.shape
        mid = width // 2
        centerHeight = height // 2

        if (self.roiHeight is not None):
            minRoiHeight = centerHeight - self.roiHeight
            maxRoiHeight = centerHeight + self.roiHeight
        else:
            minRoiHeight = 0
            maxRoiHeight = height

        left_frame = frame[minRoiHeight:maxRoiHeight, mid:, :]
        right_frame = frame[minRoiHeight:maxRoiHeight, :mid, :]

        if self.resolution in (
            Resolutions.RES_640x480_DOWNSAMPLED_8FPS,
            Resolutions.RES_640x480_DOWNSAMPLED_30FPS,
            Resolutions.RES_640x480_DOWNSAMPLED_60FPS
        ):
            left_frame = cv2.UMat(left_frame)  # convierte a UMat para usar GPU con raspberry
            right_frame = cv2.UMat(right_frame)
            roiHeight = (maxRoiHeight-minRoiHeight)

            left_resized = cv2.resize(left_frame, (640,roiHeight))
            right_resized = cv2.resize(right_frame, (640,roiHeight))

            # volver a NumPy si hace falta
            left_frame = left_resized.get()
            right_frame = right_resized.get()
            # left_frame = cv2.resize(left_frame,(640,480))
            # right_frame = cv2.resize(right_frame,(640,480))
        
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
            self.cbFrames(left, right)
        
        self.logger.error('Error getting frame')

    def startStream(self, cameraIndex, resolution):
        
        self.cap = cv2.VideoCapture(cameraIndex)
        self.resolution = resolution

        if not self.cap.isOpened:
            self.logger.error("Failed starting stream, camera not found")
            return False
        else:
            try: 
                widht , height , fps = Utils().enumToResolutionAndFps(resolution)
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
