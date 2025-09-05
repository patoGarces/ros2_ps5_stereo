import time
import cv2
from ros2_ps5_stereo.utilsClass import CameraStatusUi
from ros2_ps5_stereo.controlCamera import ControlCamera 
from ros2_ps5_stereo.controlCamera import StatusCamera
from ros2_ps5_stereo.getFrame import GetFrame

class CameraPs5Handler():

    statusCameraUi = 0

    def __init__(self, logger, resolution_enum, roi_height):

        self.logger = logger
        self.cameraResolution = resolution_enum
        self.roiHeight = roi_height

        self.controlCamera = ControlCamera(logger)
        self.getFrame =  GetFrame(logger, resolution_enum)

        self.zFilterHeight = 0
        self.zFilterThickness = 0.5
        self.depthFilter = 2.5
        self.forwardPose = 0.0
        self.statusCameraHardware = StatusCamera.CAMERA_NOT_CONNECTED

        while (self.statusCameraHardware != StatusCamera.CAMERA_CONNECTED_OK):

            self.statusCameraHardware = self.controlCamera.getCameraStatus()
            if( self.statusCameraHardware == StatusCamera.CAMERA_NOT_CONNECTED):
                self.__setStatusCameraUi(CameraStatusUi.NOT_FOUND)
            if( self.statusCameraHardware == StatusCamera.CAMERA_CONNECTED_PENDING_FW):
                self.__setStatusCameraUi(CameraStatusUi.WAITING_FW)

            time.sleep(1)

        self.__setStatusCameraUi(CameraStatusUi.CONNECTED)        
        self.startVideoStream()

    def closeEvent(self, event):
        self.video_capture.release()  # Liberar la captura de la cámara al cerrar la aplicación

    def startVideoStream(self):
        try:
            cameraIndex = self.controlCamera.getCameraIndex()
            if (cameraIndex is not None):
                self.getFrame.startStream(cameraIndex, self.cameraResolution)
            else:
                self.logger.info('Camera index no encontrado')
        except Exception as error :
            self.logger.error('error', error)
            return False
        else:
            return True

    def stopVideoStream(self):
        try:
            self.getFrame.stopStream()
        except Exception as error :
            self.logger.error('error', error)

    def setCbFrames(self, cb):
        self.getFrame.setCbFrames(cb)

    def __setStatusCameraUi(self,status: CameraStatusUi):
        self.statusCameraUi = status
        self.logger.debug('Status camera: ' + status.value)

        if (status == CameraStatusUi.WAITING_FW):
            self.logger.info("Cargando FW")
            self.__loadFirmware()
        elif (status == CameraStatusUi.CONNECTED):
            self.logger.info("Conectada, iniciando stream")
        elif (status == CameraStatusUi.NOT_FOUND):
            self.logger.info("Buscando camara")
        elif (status == CameraStatusUi.STREAM_RUNNING):
            self.logger.info("Stream corriendo")

    def __loadFirmware(self):
        result = self.controlCamera.loadFirmwareCamera()
        if (result != None):
            if(result == True):
                self.__setStatusCameraUi(CameraStatusUi.CONNECTED)
            else:
                self.logger.info('result: ' + str(result))
                self.__setStatusCameraUi(CameraStatusUi.ERROR)

    
# if __name__ == "__main__":

    # npRet = np.load('DepthParams/param_ret.npy')
    # npK = np.load('DepthParams/param_K.npy')
    # npDist = np.load('DepthParams/param_dist.npy')
    # depthMapProcessor = DepthMapProcessor(npRet,npK,npDist)

    # logging.basicConfig(level=logging.NOTSET)

    # DepthmapServerHeadless(depthMapProcessor)
    # cameraPs5Handler = CameraPs5Handler()
    # queueframeProcessed = cameraPs5Handler.getQueueFrames()

    # while True:
        # try:
        #     # Obtiene los frames de la cola de visualización
        #     frames = queueframeProcessed.get(timeout=1)

        #     frameR,frameL = frames

        #     cv2.imshow('frameL', frameR)
        #     # queueframeProcessed.updateFrames(frames)

        #     # Salir si se presiona 'q'
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break

        # except queue.Empty:
        #     print("No frames received, retrying...")
        #     continue

    # cv2.destroyAllWindows()

