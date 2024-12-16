def spin_and_map():
    global zRotationRaw, distance, rawPoints, robot

    # Speed for spinning
    spin_speed = 0.1
    points_collected = []

    # Capture the initial zRotation
    start_rotation = zRotationRaw

    # Start spinning the robot
    robot.left_motor.value = spin_speed
    robot.right_motor.value = -spin_speed

    print("Spinning and mapping...")

    # Number of full rotations to make
    rotations_to_make = 3
    target_rotation = start_rotation + 360 * rotations_to_make  # Target is 3 full rotations (1080 degrees)

    while True:
        # Continuously collect sensor data while spinning
        findPointValue(rawPoints, zRotationRaw, distance)

        # Collect the data point
        points_collected.append((zRotationRaw, distance))

        # Check if the robot has completed one full rotation (360 degrees)
        if zRotationRaw >= target_rotation:
            break

        time.sleep(0.1)  # Add delay to manage data collection rate

    # Stop the robot after spinning
    robot.stop()

    print("Spinning complete. Mapping points...")
    
    # Process raw points into the map
    for point in rawPoints:
        x, y = point
        mapped_x = x + offset[0]
        mapped_y = y + offset[1]
        map.append((mapped_x, mapped_y))
    
    print("Mapping complete.")

# Sensor input thread
thread = threading.Thread(target=getSensorInput)
thread.daemon = True
thread.start()
time.sleep(1)

# Robot actions
print("Starting spin and mapping...")
spin_and_map()
print("Done")
