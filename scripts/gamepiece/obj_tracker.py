from model_tester import ONNXDetector
import constants
import cv2
from dataclasses import dataclass
from typing import Deque
from collections import deque
import math
from typing import Optional

@dataclass(frozen=True)
class BoundingBox:
    x1: int
    y1: int
    x2: int
    y2: int
    cls: int

    @property
    def area(self) -> int:
        return max(0, self.x2 - self.x1) * max(0, self.y2 - self.y1)

    @property
    def center(self) -> tuple[int, int]:
        return (
            (self.x1 + self.x2) // 2,
            (self.y1 + self.y2) // 2,
        )

@dataclass
class Track:
    history: Deque[BoundingBox]
    missed_frames: int = 0
    x_vel: float = -1
    y_vel: float = -1
    y_accel: float = -1
    predicted_next: Optional[BoundingBox] = None

    @property
    def last(self) -> BoundingBox:
        return self.history[-1]

    @property
    def cls(self) -> int:
        return self.last.cls

    def predicted_next_pos(self) -> Optional[BoundingBox]:
        if not self.predicted_next_pos:
            return None
        dt = self.missed_frames + 1
        y_vel_pred = self.y_vel + self.y_accel * self.missed_frames
        return BoundingBox(self.last.x1 + int(self.x_vel * dt), self.last.y1 + int(y_vel_pred * dt), self.last.x2 + int(self.x_vel * dt), self.last.y2 + int(y_vel_pred * dt), self.cls)

    def update_physics(self):
        if len(self.history) == 1:
            return
        else:
            prev_vel_weight_x: float = min(0.5, (len(self.history) - 1)/20) #TODO tune
            prev_vel_weight_y: float = min(0.0, (len(self.history) - 1)/20) #TODO tune
            old_y_vel = self.y_vel
            self.x_vel = prev_vel_weight_x * self.x_vel + (1 - prev_vel_weight_x) * (self.history[-1].center[0] - self.history[-2].center[0])
            self.y_vel = prev_vel_weight_y * self.y_vel + (1 - prev_vel_weight_y) * (self.history[-1].center[1] - self.history[-2].center[1])
            prev_accel_weight: float = min(0.5, (len(self.history) - 2)/20)
            self.y_accel = prev_accel_weight * self.y_accel + (1 - prev_accel_weight) * (self.y_vel - old_y_vel)
            self.predicted_next = self.predicted_next_pos()

def compute_iou(a: BoundingBox, b: BoundingBox) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)

    inter_area = max(0, x2 - x1) * max(0, y2 - y1)
    union_area = a.area + b.area - inter_area

    return inter_area / union_area if union_area > 0 else 0.0

def parse_detections(raw) -> list[BoundingBox]:
    boxes = []
    for x1, y1, x2, y2, conf, cls in raw:
        if x1 == 0 and x2 == 0:
            continue
        boxes.append(BoundingBox(x1, y1, x2, y2, cls))
    return boxes

def run_tracking():
    MIN_DIST = -1 # TODO tune
    MIN_IOU = 0.1 # TODO tune

    detector = ONNXDetector(constants.MODEL_PATH)

    cap = cv2.VideoCapture("datasets/videos/480psnipped.mp4")

    object_position_histories: list[Track] = [] 

    counter = 0

    while (True):
        counter += 1
        ok, frame = cap.read()
        if not ok:
            print("Done")
            break
        print(f"Frame {counter}\tObjects: {len(object_position_histories)}")

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        raw_detections = detector.detect(rgb)
        detections = parse_detections(raw_detections)

        claimed = [False] * len(object_position_histories)
        num_possible_objects = len(object_position_histories)
        
        for object_pos_history in object_position_histories:
            next_pos_pred = object_pos_history.predicted_next
            if not next_pos_pred:
                continue
            cv2.rectangle(frame, (next_pos_pred.x1, next_pos_pred.y1), (next_pos_pred.x2, next_pos_pred.y2), (0, 0, 255), 4)

        for tracked in object_position_histories:
            tracked.missed_frames += 1

        for box in detections:
            if box.area == 0:
                continue
            
            closest_obj_index_by_dist: Optional[int] = None
            closest_obj_index_by_iou: Optional[int] = None
            largest_iou = 0
            lowest_dist = 100000
            for i in range(num_possible_objects):
                if claimed[i] or object_position_histories[i].cls != box.cls:
                    continue
                predicted_next = object_position_histories[i].predicted_next
                if not predicted_next:
                    iou = compute_iou(box, object_position_histories[i].last)
                    if iou > largest_iou:
                        largest_iou = iou
                        closest_obj_index = i
                else:
                    distance = math.hypot(box.center[0] - predicted_next.center[0], box.center[1] - predicted_next.center[1])
                    if (distance < MIN_DIST and distance < lowest_dist):
                        
            if closest_obj_index != -1:
                object_position_histories[closest_obj_index].history.append(box)
                object_position_histories[closest_obj_index].missed_frames = 0
                claimed[closest_obj_index] = True
            else:
                object_position_histories.append(Track(deque(maxlen=15), 0))
                object_position_histories[-1].history.append(box)
                claimed.append(True)
        detector.draw_detections(frame, raw_detections)
        object_position_histories = [x for x in object_position_histories if x.missed_frames <= 3]
        for object_pos_history in object_position_histories:
            object_pos_history.update_physics()
            if len(object_pos_history.history) < 2:
                continue
            for i in range(1, len(object_pos_history.history)):
                new_center = object_pos_history.history[i].center
                old_center = object_pos_history.history[i-1].center
                cv2.line(frame, old_center, new_center, (0, 255, 0), 4)
        cv2.imshow("Test", frame)
        cv2.waitKey(0)
    cv2.destroyAllWindows() 
            
if __name__=="__main__":
    run_tracking()
