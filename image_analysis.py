import cv2

from numpy import pi

def find_target_block(bgr_image, hsv_min, hsv_max):
    height, width = bgr_image.shape[:2]
    half_height = height/2
    half_width = width/2
    marked_image = cv2.warpAffine(
        src=bgr_image,
        M=cv2.getRotationMatrix2D(center=(half_width, half_height), angle=180, scale=1),
        dsize=(width, height)
    )
    hsv_image = cv2.cvtColor(marked_image, cv2.COLOR_BGR2HSV)

    masked_image = cv2.inRange(hsv_image, hsv_min, hsv_max)
    _, thresh = cv2.threshold(masked_image, 127, 255, 0)
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    have_block = (len(contours) > 0)

    enclosed_circle_area = None
    degrees_away = None
    if have_block:
        (x,y),r = cv2.minEnclosingCircle(max(contours, key=cv2.contourArea))
        cv2.circle(marked_image, (int(x), int(y)), int(r), (255,0,255), 2)
        enclosed_circle_area = (pi * r * r)
        degrees_away = ((x - half_width) * 0.061)

    return marked_image, have_block, enclosed_circle_area, degrees_away
