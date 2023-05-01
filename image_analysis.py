import cv2
from numpy import zeros, uint8

from gc_constants import (
    LOCALIZE_TARGET_BLOCK_IMAGE_TOP_CUTOFF,
    LOCALIZE_TARGET_BLOCK_IMAGE_BOTTOM_CUTOFF,
)

def localize_target_block(bgr_image, hsv_min, hsv_max):
    height, width = bgr_image.shape[:2]
    half_height = height/2
    half_width = width/2
    marked_image = cv2.warpAffine(
        src=bgr_image,
        M=cv2.getRotationMatrix2D(center=(half_width, half_height), angle=180, scale=1),
        dsize=(width, height)
    )
    marked_image = marked_image[LOCALIZE_TARGET_BLOCK_IMAGE_TOP_CUTOFF:LOCALIZE_TARGET_BLOCK_IMAGE_BOTTOM_CUTOFF, :]
    hsv_image = cv2.cvtColor(marked_image, cv2.COLOR_BGR2HSV)

    masked_image = cv2.inRange(hsv_image, hsv_min, hsv_max)
    _, thresh = cv2.threshold(masked_image, 127, 255, 0)
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    have_block = (len(contours) > 0)

    block_pixel_area = None
    degrees_away = None
    if have_block:
        largest_contour = max(contours, key=cv2.contourArea)
        (x,y),r = cv2.minEnclosingCircle(largest_contour)
        cv2.circle(marked_image, (int(x), int(y)), int(r), (255,0,255), 2)
        contour_only = cv2.drawContours(zeros(masked_image.shape, uint8), [largest_contour], 0, 255, cv2.FILLED)
        cv2.imshow("Contour only", contour_only)
        block_pixel_area = cv2.countNonZero(contour_only)
        degrees_away = ((x - half_width) * 0.061) * -1

    return marked_image, have_block, block_pixel_area, degrees_away
