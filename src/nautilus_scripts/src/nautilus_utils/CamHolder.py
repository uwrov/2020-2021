class CamHolder:
    def __init__(self):
        self._cam1 = None
        self._cam2 = None
    
    def update_frame(self, frame, target_cam):
        if target_cam == 1:
            self._cam1 = frame
        else:
            self._cam2 = frame
    
    def get_frame(self, target_cam):
        if target_cam == 1:
            return self._cam1
        else:
            return self._cam2