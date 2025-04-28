import cv2
from ultralytics import YOLO

net = None
trt_model = None


class person_to_track():
    def __init__(self, Center, Left, Right, Top, Bottom):
        self.Center = Center
        self.Left = Left  # x
        self.Right = Right  # x
        self.Top = Top  # y
        self.Bottom = Bottom  # y


def initialize_detector():
    global camera, trt_model
    trt_model = YOLO("./yolo11n.pt")

    trt_model.classes = [0]
    print("yolo ok")
    camera_id = "/dev/video4"
    camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    print("camera ok")


def get_image_size():
    width = camera.get(cv2.CAP_PROP_FRAME_WIDTH)  # float `width`
    height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
    return int(width), int(height)


def close_camera():
    camera.release()


def get_detections(conf_threshold=0.5):
    person_detections = []
    if camera.isOpened():
        ret_val, frame = camera.read()
        fps = camera.get(cv2.CAP_PROP_FPS)

        results = trt_model(frame, verbose=False)
        for result in results:
            boxes = result.boxes
            for box in boxes:
                if box.cls == 0 and box.conf >= conf_threshold:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    temp_person = person_to_track(Center=(int(center_x), int(center_y)),
                                                  Left=x1,
                                                  Right=x2,
                                                  Top=y1,
                                                  Bottom=y2,
                                                  )
                    person_detections.append(temp_person)
    return person_detections, fps, frame
