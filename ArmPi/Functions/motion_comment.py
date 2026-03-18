# Motion loop (runs in a background thread in the vendor code)
# Inputs from perception (globals): detect_color, world_X, world_Y, world_x, world_y, rotation_angle
# Flags: track, start_pick_up, first_move, unreachable, action_finish

# Drop-off bins by color (world coordinates)
coordinate = {
    'red':   (-14.5, 11.5, 1.5),
    'green': (-14.5,  5.5, 1.5),
    'blue':  (-14.5, -0.5, 1.5),
}

# 1) First time a block is detected and considered stable:
#    - move above the block at a safe height
#    - this "locks in" the target before tracking begins
if first_move and start_pick_up:
    action_finish = False
    set_rgb(detect_color)           # LED feedback
    setBuzzer(0.1)                  # beep feedback
    # Move above target (note: vendor uses world_Y - 2 offset)
    result = AK.setPitchRangeMoving((world_X, world_Y - 2, 5), -90, -90, 0)
    unreachable = (result is False)
    if not unreachable:
        time.sleep(result[2] / 1000)  # wait until move completes

    # Clear pick trigger; next loop will track or pick
    start_pick_up = False
    first_move = False
    action_finish = True

# 2) Tracking phase: follow moving object at constant height
elif not first_move and not unreachable:
    if track:
        # quick, low-latency moves (20 ms command time)
        AK.setPitchRangeMoving((world_x, world_y - 2, 5), -90, -90, 0, 20)
        time.sleep(0.02)
        track = False

    # 3) Pick phase: object is stable long enough -> execute pick-and-place
    if start_pick_up:
        action_finish = False

        # Open gripper
        Board.setBusServoPulse(1, servo1 - 280, 500)

        # Rotate wrist to match block orientation
        servo2_angle = getAngle(world_X, world_Y, rotation_angle)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.8)

        # Descend to pick height
        AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)
        time.sleep(2)

        # Close gripper
        Board.setBusServoPulse(1, servo1, 500)
        time.sleep(1)

        # Lift
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)
        time.sleep(1)

        # Move to color bin (above)
        result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)
        time.sleep(result[2] / 1000)

        # Rotate wrist for placement
        servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)

        # Descend, place, open gripper
        AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
        AK.setPitchRangeMoving(coordinate[detect_color], -90, -90, 0, 1000)
        Board.setBusServoPulse(1, servo1 - 200, 500)  # open to release

        # Retract and home
        AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
        initMove()

        # Reset state for next cycle
        detect_color = 'None'
        first_move = True
        start_pick_up = False
        action_finish = True
        set_rgb(detect_color)