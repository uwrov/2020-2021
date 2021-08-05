import numpy as np
from pyyolo import BBox
from collections import OrderedDict

class TrackedObject:
    def __init__(self, timestamp: int, bbox: BBox):
        self.initial_timestamp = timestamp
        self.max_timestamp = timestamp
        self.nframes = 1
        self.max_bbox = bbox
        self.curr_bbox = bbox
        self.centroid = self._find_centroid(bbox)

    def update(self, timestamp:int, bbox: BBox) -> None:
        self.centroid = self._find_centroid(bbox)
        self.nframes += 1
        self.curr_bbox = bbox
        if self._area(bbox) > self._area(self.max_bbox):
            self.max_bbox = bbox
            self.max_timestamp = timestamp

    def _find_centroid(self, bbox: BBox) -> np.array:
        xmin, ymin, xmax, ymax = bbox.to_xyxy()
        return np.array([np.mean([xmin, xmax]), np.mean([ymin, ymax])])

    def _area(self, bbox: BBox):
        return bbox.w * bbox.h

    def __str__(self):
        return 'intial_timestamp: ' + str(self.initial_timestamp) + '\nmax_timestamp: ' + str(self.max_timestamp) + '\nnframes: ' + str(self.nframes) + \
                '\nmax_bbox: ' + str(self.max_bbox) + '\ncentroid: ' + str(self.centroid) + '\ncurr_bbox: ' + str(self.curr_bbox)

class CentroidTracker:
    def __init__(self, max_disappeared=15, max_distance=5):
        self.next_id = 0
        self.output_log = OrderedDict()
        self.registered = OrderedDict()
        self.disappeared = OrderedDict()
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance

    def update(self, detected: list, timestamp: int) -> None:
        # Take in a list of detected bounding boxes from our yolo detector
        # update the registered centroids we're keeping track of

        if len(self.registered) == 0:
            # initial case, register all detected objects
            for bbox in detected:
                self.registered[self.next_id] = TrackedObject(timestamp, bbox)
                self.next_id += 1
        else:
            # Try to match detected objects to what we have registered
            unmatched = set(self.registered.keys())
            new_objects = []
            for i, bbox in enumerate(detected):
                nn = self._find_neighbor(bbox)
                if nn in unmatched:
                    unmatched.remove(nn)
                    self.disappeared[nn] = 0
                    self.registered[nn].update(timestamp, bbox)
                elif nn == -1:
                    new_objects.append(i)

            # register a new object
            for i in new_objects:
                self.registered[self.next_id] = TrackedObject(timestamp, detected[i])
                self.next_id += 1

            # deregister an old object which has been gone for too long
            for id in unmatched:
                if id not in self.disappeared.keys():
                    self.disappeared[id] = 0
                self.disappeared[id] += 1
                if self.disappeared[id] > self.max_disappeared:
                    self._deregister(id)
        return

    def signal_end(self) -> OrderedDict:
        for id in list(self.registered.keys()):
            self._deregister(id)

        return self.output_log

    def get_registered_objects(self) -> OrderedDict:
        return self.registered

    def _deregister(self, id: int):
        self.output_log[id] = self.registered.pop(id)

    def _find_neighbor(self, bbox: BBox) -> int:
        min_idx = -1
        min_dist = 100000
        c = self._find_centroid(bbox)

        for idx in self.registered.keys():
            obj = self.registered[idx]
            dist = np.linalg.norm(c - obj.centroid)
            if dist < min_dist and dist < self.max_distance:
                min_idx = idx
                min_dist = dist

        return min_idx

    def _find_centroid(self, bbox: BBox) -> np.array:
        xmin, ymin, xmax, ymax = bbox.to_xyxy()
        return np.array([np.mean([xmin, xmax]), np.mean([ymin, ymax])])