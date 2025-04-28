import sys
sys.path.insert(1, 'modules')

import cv2

import modules.lidar as lidar
import modules.detector as detector
import modules.vision as vision

print("setting up detector")
detector.initialize_detector()
lidar.connect_lidar()

image_width, image_height = detector.get_image_size()
image_center = (image_width / 2, image_height / 2)

def prepare_visualisation(
        lidar_distance,
        person_center,
        person_to_track,
        image,
        yaw_command,
        x_delta,
        y_delta,
        fps,
        velocity_x_command,
        lidar_on_target
):
    lidar_vis_x = image_width - 50
    lidar_vis_y = image_height - 50
    lidar_vis_y2 = int(image_height - lidar_distance * 200)
    cv2.line(image, (lidar_vis_x,lidar_vis_y), (lidar_vis_x, lidar_vis_y2), (0, 255, 0), thickness=10, lineType=8, shift=0)
    cv2.putText(image, "distance: " + str(round(lidar_distance,2)), (image_width - 300, 200), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA)

    #draw path
    cv2.line(image, (int(image_center[0]), int(image_center[1])), (int(person_center[0]), int(person_center[1])), (255, 0, 0), thickness=10, lineType=8, shift=0)

    #draw bbox around target
    cv2.rectangle(image,(int(person_to_track.Left),int(person_to_track.Bottom)), (int(person_to_track.Right),int(person_to_track.Top)), (0,0,255), thickness=10)

    #show drone center
    cv2.circle(image, (int(image_center[0]), int(image_center[1])), 20, (0, 255, 0), thickness=-1, lineType=8, shift=0)

    #show trackable center
    cv2.circle(image, (int(person_center[0]), int(person_center[1])), 20, (0, 0, 255), thickness=-1, lineType=8, shift=0)

    #show stats
    cv2.putText(image, "fps: " + str(round(fps,2)) + " yaw: " + str(round(yaw_command,2)) + " forward: " + str(round(velocity_x_command,2)) , (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA)
    cv2.putText(image, "lidar_on_target: " + str(lidar_on_target), (50, 100), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA)
    cv2.putText(image, "x_delta: " + str(round(x_delta,2)) + " y_delta: " + str(round(y_delta,2)), (50, 150), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA)

    return image

while True:
    detections, fps, image = detector.get_detections()
    cv2.putText(image, str(fps), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)

    if len(detections) > 0:
        person_to_track = detections[0]
        person_center = person_to_track.Center  # get center of person to track
        x_delta = vision.get_single_axis_delta(image_center[0], person_center[0])  # get x delta
        y_delta = vision.get_single_axis_delta(image_center[1], person_center[1])  # get y delta
        lidar_on_target = vision.point_in_rectangle(image_center, person_to_track.Left, person_to_track.Right,
                                                    person_to_track.Top,
                                                    person_to_track.Bottom)  # check if lidar is pointed on target

        image = prepare_visualisation(
            lidar.read_lidar_distance(),
            person_center,
            person_to_track,
            image,
            0,
            x_delta,
            y_delta,
            fps,
            0,
            lidar_on_target
        )

    cv2.imshow("out", image)
    cv2.waitKey(1)
