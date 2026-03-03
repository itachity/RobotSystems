# --- Perception helpers: contour selection ---
def getAreaMaxContour(contours):
    """
    From a list of contours, pick the largest contour whose area is above a minimum threshold.
    This filters small noisy blobs from lighting/reflections.
    Returns (area_max_contour, contour_area_max).
    """
    contour_area_max = 0.0
    area_max_contour = None

    for c in contours:
        area = abs(cv2.contourArea(c))
        # Track maximum-area contour
        if area > contour_area_max:
            contour_area_max = area
            # Only accept as a "real object" if sufficiently large
            if area > 300:
                area_max_contour = c

    return area_max_contour, contour_area_max


# --- Perception pipeline: frame -> detection -> "stable pick" trigger ---
def run(img):
    """
    Perception pipeline:
      1) Preprocess frame (resize + blur)
      2) Optional ROI masking (reduce search to last-known region)
      3) Convert to LAB color space
      4) Threshold LAB for target color(s)
      5) Morphological open/close to remove speckle + fill gaps
      6) Find contours, select the largest blob
      7) Fit min-area rectangle, get pixel center + rotation
      8) Convert pixel center -> world coordinates
      9) Apply stability test (object still for N seconds)
     10) If stable, output world_X, world_Y, rotation_angle and raise start_pick_up flag
    """

    global roi, rect, count, track, get_roi, center_list
    global __isRunning, unreachable, detect_color, action_finish
    global rotation_angle, last_x, last_y
    global world_X, world_Y, world_x, world_y
    global start_count_t1, t1, start_pick_up, first_move

    if not __isRunning:
        return img

    img_copy = img.copy()

    # Draw crosshair for debugging
    h, w = img.shape[:2]
    cv2.line(img, (0, h // 2), (w, h // 2), (0, 0, 200), 1)
    cv2.line(img, (w // 2, 0), (w // 2, h), (0, 0, 200), 1)

    # 1) Resize to fixed size used by calibration mapping
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)

    # 2) Blur to reduce pixel noise before thresholding
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

    # 3) Optional ROI mask:
    # If we already have an ROI from last detection, restrict processing to that ROI.
    # (Your tracking code only applies ROI mask during start_pick_up; sorting code does it earlier.)
    if get_roi and start_pick_up:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)

    # 4) Convert to LAB for robust color segmentation
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

    # 5) Threshold + morphology + contour selection for target color(s)
    areaMaxContour = None
    area_max = 0

    if not start_pick_up:
        for color_name in color_range:                     # LABConfig.color_range dict
            if color_name in __target_color:               # only process chosen colors
                detect_color = color_name
                lower, upper = color_range[detect_color]   # LAB bounds

                frame_mask = cv2.inRange(frame_lab, lower, upper)

                # Remove small speckles and close small holes
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))

                # Find connected components (contours)
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

                # Largest contour is our candidate block
                c, a = getAreaMaxContour(contours)
                if a > area_max:
                    areaMaxContour = c
                    area_max = a

        # If object blob area is large enough, treat as valid detection
        if area_max > 2500 and areaMaxContour is not None:
            # 6) Fit rectangle and compute ROI
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            roi = getROI(box)
            get_roi = True

            # 7) Pixel center of rectangle -> image center
            img_centerx, img_centery = getCenter(rect, roi, size, square_length)

            # 8) Convert pixel coordinates -> world coords (cm) using calibration config
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size)

            # Debug overlay: rectangle + world coords label
            cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
            cv2.putText(img, f"({world_x:.2f},{world_y:.2f})",
                        (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)

            # 9) Stability test:
            # Compare current world coord with last coord; if moved little for long enough -> stable.
            distance = math.sqrt((world_x - last_x)**2 + (world_y - last_y)**2)
            last_x, last_y = world_x, world_y
            track = True

            if action_finish:
                if distance < 0.3:
                    center_list.extend((world_x, world_y))
                    count += 1
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()

                    # Stable long enough -> trigger pick
                    if time.time() - t1 > 1.5:
                        rotation_angle = rect[2]
                        start_count_t1 = True

                        # Average the recent stable centers for a smoother final pick point
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)

                        # reset buffers and trigger pick
                        count = 0
                        center_list = []
                        start_pick_up = True
                else:
                    # moved too much, reset stability timer
                    t1 = time.time()
                    start_count_t1 = True
                    count = 0
                    center_list = []

    return img