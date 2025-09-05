from enum import Enum

class CameraStatusUi(Enum):
    CONNECTING = "Conectando camara"
    CONNECTED = "Camara Conectada"
    WAITING_FW = "Camara esperando fw"
    NOT_FOUND = "Camara no encontrada"
    STREAM_INIT = "Iniciando Stream"
    STREAM_RUNNING = "Stream corriendo"
    ERROR = "Error camara"

class Resolutions(Enum):
    RES_640x480_DOWNSAMPLED_8FPS = 0
    RES_640x480_DOWNSAMPLED_30FPS = 1
    RES_640x480_DOWNSAMPLED_60FPS = 2
    RES_1280x800_8FPS = 3
    RES_1280x800_30FPS = 4
    RES_1280x800_60FPS = 5
    RES_1920x1080_8FPS = 6
    RES_1920x1080_30FPS = 7

class Utils:

    def enumResolutionsToPeriod(self, resolution: Resolutions):
        mapping = {
            Resolutions.RES_640x480_DOWNSAMPLED_8FPS:  (0.125),
            Resolutions.RES_640x480_DOWNSAMPLED_30FPS: (0.03333),
            Resolutions.RES_640x480_DOWNSAMPLED_60FPS: (0.016666),
            Resolutions.RES_1280x800_8FPS:             (0.125),
            Resolutions.RES_1280x800_30FPS:            (0.03333),
            Resolutions.RES_1280x800_60FPS:            (0.016666),
            Resolutions.RES_1920x1080_8FPS:            (0.125),
            Resolutions.RES_1920x1080_30FPS:           (0.03333),
        }

        if resolution not in mapping:
            raise ValueError(f"Resolución no soportada: {resolution}")

        return mapping[resolution]

    def enumToResolutionAndFps(self, resolution: Resolutions):
        
        mapping = {
            Resolutions.RES_640x480_DOWNSAMPLED_8FPS:   (1280*2, 800, 8),   # La resolucion es la misma, posteriormente hago el downsample
            Resolutions.RES_640x480_DOWNSAMPLED_30FPS:  (1280*2, 800, 30),
            Resolutions.RES_640x480_DOWNSAMPLED_60FPS:  (1280*2, 800, 60),
            Resolutions.RES_1280x800_8FPS:              (1280*2, 800, 8),
            Resolutions.RES_1280x800_30FPS:             (1280*2, 800, 30),
            Resolutions.RES_1280x800_60FPS:             (1280*2, 800, 60),
            Resolutions.RES_1920x1080_8FPS:             (1920*2, 1080, 8),
            Resolutions.RES_1920x1080_30FPS:            (1920*2, 1080, 30),
        }

        if resolution not in mapping:
            raise ValueError(f"Resolución no soportada: {resolution}")

        width, height, fps = mapping[resolution]

        return width, height, fps