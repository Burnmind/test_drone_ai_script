import numpy as np
import cv2 as cv
import pyrealsense2 as rs

debug = True

pipeline = rs.pipeline()
config = rs.config()
image_width = 640
image_height = 480
config.enable_stream(rs.stream.depth, image_width, image_height, rs.format.z16, 30)
config.enable_stream(rs.stream.color, image_width, image_height, rs.format.bgr8, 30)
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
color_sensor = profile.get_device().query_sensors()[1]
depth_scale = depth_sensor.get_depth_scale()
align_to = rs.stream.color
align = rs.align(align_to)
decimation = rs.decimation_filter()
decimation.set_option(rs.option.filter_magnitude, 2)
depth_to_disparity = rs.disparity_transform(True)
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 2)
spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
spatial.set_option(rs.option.filter_smooth_delta, 20)
temporal = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
temporal.set_option(rs.option.filter_smooth_delta, 20)
disparity_to_depth = rs.disparity_transform(False)
hole_filling = rs.hole_filling_filter()

previous_zDepth = 1


def read_distance():
    """ Функция для определения расстояния до препятствия в центре кадра

   """
    # Get frameset of color and depth
    frames = pipeline.wait_for_frames()

    # Apply the filters to depth frames
    filtered_depth = decimation.process(frames.get_depth_frame())
    filtered_depth = depth_to_disparity.process(filtered_depth)
    filtered_depth = spatial.process(filtered_depth)
    filtered_depth = temporal.process(filtered_depth)
    filtered_depth = disparity_to_depth.process(filtered_depth)
    filtered_depth = hole_filling.process(filtered_depth)
    # filtered_depth = threshold_filter.process(filtered_depth)

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    # if not aligned_depth_frame or not color_frame:
    #     continue

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    center_x = int(image_width/2)
    center_y = int(image_height/2)

    global previous_zDepth
    zDepth = aligned_depth_frame.get_distance(center_x, center_y) / depth_scale / 1000
    if zDepth == 0:
        zDepth = previous_zDepth
    if debug:
        cv.putText(color_image, f"{zDepth} ({center_x},{center_y})", (center_x, center_y),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
                   cv.LINE_AA)
        cv.namedWindow('Recorder Realsense', cv.WINDOW_AUTOSIZE)
        cv.imshow('Recorder Realsense', color_image)
        key = cv.waitKey(1)

        # if 'esc' button pressed, escape loop and exit program
        if key == 27:
            cv.destroyAllWindows()

    previous_zDepth = zDepth
    return zDepth


def get_obstacles_matrix(clipped_distance: int = 1) -> np.ndarray:
    """ Функция для определения наличия препятствий

    Изображение с камеры делится на секторы. Получается матрица размером 3x3. Если в секторе есть препятствие, то значение элемента матрицы становится True

    Пример вывода если есть препятствие с левой стороны:
    [[ True  True  True] [False False False] [ False False False]]


    :param int clipped_distance: Расстояние на котором считаем любой предмет препятствием. Не больше 4.

    :return: numpy.array размером 3x3
   """
    matrix = np.array([[False, False, False], [False, False, False], [False, False, False]])

    clipping_distance_in_meters = clipped_distance
    clipping_distance = clipping_distance_in_meters / depth_scale

    def to_simple_matrix(matrix, obstacles_found):
        arr = np.array(obstacles_found)
        y_parts = np.array_split(arr, 3, axis=0)  # разбили на 3 строки высотой 160 и шириной 640
        for x, y_part in enumerate(y_parts):
            x_parts = np.array_split(y_part, 3, axis=1)  # разбили строку на 3 столбика высотой 160 и шириной 214
            for y, x_part in enumerate(x_parts):
                for row in x_part:
                    for have_objects in row:
                        if have_objects:
                            matrix[y][x] = have_objects or matrix[y][x]
                            break
        return (matrix)


    # Get frameset of color and depth
    frames = pipeline.wait_for_frames()

    # Apply the filters to depth frames
    filtered_depth = decimation.process(frames.get_depth_frame())
    filtered_depth = depth_to_disparity.process(filtered_depth)
    filtered_depth = spatial.process(filtered_depth)
    filtered_depth = temporal.process(filtered_depth)
    filtered_depth = disparity_to_depth.process(filtered_depth)
    filtered_depth = hole_filling.process(filtered_depth)
    # filtered_depth = threshold_filter.process(filtered_depth)

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    # if not aligned_depth_frame or not color_frame:
    #     continue

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Remove background - Set pixels further than clipping_distance to white
    white_color = 255
    # depth image is 1 channel, color is 3 channels
    depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
    bg_removed_color = np.where((depth_image_3d > clipping_distance) | \
                                (depth_image_3d <= 0), white_color, color_image)
    bg_removed_depth = np.where((depth_image > clipping_distance) | (depth_image <= 0), white_color,
                                depth_image)
    obstacles_found = np.where((depth_image > clipping_distance) | (depth_image <= 0), False, True)
    matrix = to_simple_matrix(matrix, obstacles_found)
    #matrix = np.flip(matrix.T, 1)

    if debug:
        image_width = len(obstacles_found[0]) # Ширина изображения
        image_height = len(obstacles_found) # Высота изображения
        he = int(image_height / 3)
        we = int(image_width / 3)

        # for y, row in enumerate(matrix):
        #     for x, col in enumerate(matrix[y]):
        #         cv.putText(bg_removed_color, f"{col} ({2-x},{y})", ((2-x)*we+int(we/2), y*he+int(he/2)), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
        #                    cv.LINE_AA)
        for x, col in enumerate(matrix):
            # print("x "+str(x))
            for y, el in enumerate(matrix[x]):
                # print("y " + str(y))
                cv.putText(bg_removed_color, f"{el} ({x},{y})", (x*we+int(we/2), y*he+int(he/2)), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
                           cv.LINE_AA)
        # Render images
        depth_colormap = cv.applyColorMap(
            cv.convertScaleAbs(bg_removed_depth, alpha=0.09), cv.COLORMAP_JET)
        # images = np.hstack((bg_removed_color, depth_colormap))
        images = bg_removed_color
        cv.namedWindow('Recorder Realsense', cv.WINDOW_AUTOSIZE)
        cv.imshow('Recorder Realsense', images)
        key = cv.waitKey(1)

        # if 'esc' button pressed, escape loop and exit program
        if key == 27:
            cv.destroyAllWindows()

    return (matrix)


if debug:
    while True:
        matrix = read_distance()
        try:
            import os
            clear = lambda: os.system('cls')
            print(f"{matrix}")
            clear()
        except:
            pass