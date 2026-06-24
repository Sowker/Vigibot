import cv2
import numpy as np
from picamera2 import Picamera2

def get_direction():
    # Initialize Picamera2
    picam = Picamera2()
    picam.configure(picam.create_preview_configuration())
    picam.start()

    try:
        # capture_array() returns the array directly, NOT a tuple!
        prev_frame = picam.capture_array()
    except Exception as e:
        picam.stop()
        raise RuntimeError(f"Could not read first frame: {e}")

    # Get height and width of the camera feed
    h, w = prev_frame.shape[:2]

    # Define range for white color in HSV
    # (Hue, Saturation, Value)
    lower_white = np.array([0, 0, 150])
    upper_white = np.array([179, 70, 255])

    # Define range for black color in HSV
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([255, 255, 50])

    while True:
        try:
            default_frame = picam.capture_array()
        except RuntimeError as e:
            print(f"Failed to capture frame in loop: {e}")
            break

        # Appy Gaussian Blur
        default_frame = cv2.GaussianBlur(default_frame, (15, 15), 0)

        # cv2.imshow('Default frame', default_frame)

        # ==== To Fine tune ====
        # Crop the image by 10% - on each side - to focus on the center
        # frame = default_frame[round(h*0.1):round(h-h*0.11),round(w*0.1):round(w-w*0.1)]
        frame = default_frame

        # ==== To Fine tune ====
        # Detect the white (out-layer of the arrow) to crop the image to it.
        # So we can remove unnecessary noise

        # Convert the frame from RGB to HSV format
        hsv = cv2.cvtColor(default_frame, cv2.COLOR_BGR2HSV)

        # Create a white mask to focus on what is inside the white part
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Filter the white part
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Create the matrice of white from applying the mask
        has_white_x = np.any(mask, axis=0)
        has_white_y = np.any(mask, axis=1)

        # Separate each axcis
        x_coords = np.where(has_white_x)[0]
        y_coords = np.where(has_white_y)[0]

        x_min: int
        y_min: int
        x_max: int
        y_max: int

        # Verify that the matrice x and y hold white because we want to detect them
        if not(x_coords.size > 0 and y_coords.size > 0):
            print("No white detected on the image.")
            # cv2.imshow('Raw image', default_frame)

        elif x_coords.size > 0 and y_coords.size > 0:
            x_min, x_max = x_coords[0], x_coords[-1]
            y_min, y_max = y_coords[0], y_coords[-1]

            print(f"White detected: x({x_min}-{x_max}), y({y_min}-{y_max})")
            box_width = x_max - x_min
            box_height = y_max - y_min
            if box_width < 10 or box_height < 10:
                print(f"Skipping tiny/invalid frame: x({x_min}-{x_max}), y({y_min}-{y_max})")

                # You still MUST show the camera so it doesn't freeze!
                # cv2.imshow('Raw image', default_frame)
            else :

                # Draw the rectangle box using the min/max coordinates
                default = cv2.rectangle(default_frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                # Cut the frame to only have the white
                frame = frame[y_min:y_max, x_min:x_max]

                # Convert the new cropped frame from RGB to HSV format
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # Create mask for the black color
                mask = cv2.inRange(hsv, lower_black, upper_black)

                # Filter the black part
                result = cv2.bitwise_and(frame, frame, mask=mask)

                # Display the feeds
                # cv2.imshow('Raw image', frame)
                # cv2.imshow('What is detected', mask)

                # Calculate the moments of the binary mask
                moments = cv2.moments(mask)

                # Prevent division by zero
                if moments["m00"] == 0:
                    pass
                elif moments["m00"] != 0:
                    # Calculate X and Y coordinates of the center
                    cX = int(moments["m10"] / moments["m00"])
                    cY = int(moments["m01"] / moments["m00"])

                    # Optional: Draw a red dot on the result image to visualize it
                    cv2.circle(result, (cX, cY), 5, (0, 0, 255), -1)

                    # Cut the image in half in function of the middle of the arrow
                    left_img = frame[:,:cX]
                    right_img = frame[:,cX:]

                    # To cut the image in two and visualize the cutting
                    # cv2.imshow('Right', left_img)
                    # cv2.imshow('Left', right_img)

                    # Create mask for the white color
                    mask_left = mask[:, :cX]
                    mask_right = mask[:, cX:]

                    # perform the mask on the separated image to analyze them separatly and compare them
                    res_left = cv2.bitwise_and(left_img, left_img, mask=mask_left)
                    res_right = cv2.bitwise_and(right_img, right_img, mask=mask_right)

                    # count the white to know the side of the arrow
                    black_pixel_left = cv2.countNonZero(mask_left)
                    black_pixel_right = cv2.countNonZero(mask_right)

                    if black_pixel_left > black_pixel_right:
                        print("left")
                        return "left"
                    elif black_pixel_right > black_pixel_left:
                        print("right")
                        return "right"
                    else:
                        print("Mask error")

        # Wait for 1 ms, and check if 'q' is pressed to quit
        # if cv2.waitKey(1) & 0xFF == ord('q'):
            # break

    picam.stop()
    # cv2.destroyAllWindows()