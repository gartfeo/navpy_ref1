class CameraZoomControllerSiyi(object):
    def __init__(self):
        print('tessssssssssssssssss')
        # self._siyi = SIYISDK()
        # self._siyi.connect()
        print('camera_zoom, initialized')

    def set_zoom_level(self, level):
        self._siyi.requestZoomIn()

    def get_zoom_level(self):
        return self._siyi.getZoomLevel()

    def close(self):
        self._siyi.disconnect()
