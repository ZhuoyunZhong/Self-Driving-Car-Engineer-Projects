# Import libraries
import matplotlib.image as mpimg
import os
from moviepy.editor import VideoFileClip
import cv2
import numpy as np


def grayscale(img):
    """Applies the Grayscale transform"""
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Or use BGR2GRAY if you read an image with cv2.imread()
    # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def region_of_interest(img, vertices):
    """
    Applies an image mask.
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    `vertices` should be a numpy array of integer points.
    """
    # defining a blank mask to start with
    mask = np.zeros_like(img)

    # defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    # filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    # returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines, color=[255, 0, 0], thickness=10):
    """Takes the result lines from Hough Transform as input,
    draw the final extended lane found"""
    # Acquire current lane situation
    global current_left_lane
    global current_right_lane

    # Get two of the found longest lines as dominant lines
    max_length_left_line, max_length_right_line = get_dominant_lines(img, lines)

    # Draw a smoother new extended lines
    # Left one
    # Check whether the two dominant lines are reasonable, return a combine extended line if they are,
    # else return current line.
    nx1, ny1, nx2, ny2 = check_lane(img, current_left_lane, max_length_left_line)
    # To acquire a smoother change from the current line to new one
    current_left_lane = x1, y1, x2, y2 = smoother(img, current_left_lane, nx1, ny1, nx2, ny2)
    # Draw the line
    cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    # Right one
    nx1, ny1, nx2, ny2 = check_lane(img, current_right_lane, max_length_right_line)
    current_right_lane = x1, y1, x2, y2 = smoother(img, current_right_lane, nx1, ny1, nx2, ny2)
    cv2.line(img, (x1, y1), (x2, y2), color, thickness)


def get_dominant_lines(img, lines):
    """Get two of the found longest lines as dominant lines to estimate real lane location"""
    # Image information
    xsize = img.shape[1]

    # Used for finding two dominant lane
    # Ps: [0] is the one smaller than [1]
    max_length_left = [0, 0]
    max_length_right = [0, 0]
    max_length_left_line = [[0, 0, 0, 0], [0, 0, 0, 0]]
    max_length_right_line = [[0, 0, 0, 0], [0, 0, 0, 0]]

    for line in lines:
        for x1, y1, x2, y2 in line:
            # Calculate length of the line
            length = ((y2-y1)**2 + (x2-x1)**2)**1/2
            # Calculate gradient of the line
            if x2 == x1:
                continue
            k = ((y2-y1)/(x2-x1))
            # Line gradient should line in this gradient threshold and
            # it should also lie in one of the half parts of the image
            thres_gradient = (0.5, 1.7)

            # left ones
            if (-thres_gradient[1] < k < -thres_gradient[0]) and (x1 < xsize/2):
                # Replace the shorter one
                if length > max_length_left[1]:
                    max_length_left = [max_length_left[1], length]
                    max_length_left_line = [max_length_left_line[1], [x1, y1, x2, y2]]
                elif length > max_length_left[0]:
                    max_length_left[0] = length
                    max_length_left_line[0] = x1, y1, x2, y2
            # right ones
            elif (thres_gradient[0] < k < thres_gradient[1]) and (x2 > xsize/2):
                # Replace the shorter one
                if length > max_length_right[1]:
                    max_length_right = [max_length_right[1], length]
                    max_length_right_line = [max_length_right_line[1], [x1, y1, x2, y2]]
                elif length > max_length_left[0]:
                    max_length_right[0] = length
                    max_length_right_line[0] = x1, y1, x2, y2

    return max_length_left_line, max_length_right_line


def check_lane(img, current_line, new_line):
    """This function checks new line coordinate and compare it with the current one.
    If new line is reasonable, return it, else return the current one."""
    # Image information
    ysize = img.shape[0]
    xsize = img.shape[1]

    # New line positions
    nx1, ny1, nx2, ny2 = new_line[0]
    nx3, ny3, nx4, ny4 = new_line[1]
    cv2.line(img, (nx1, ny1), (nx2, ny2), [255, 0, 255], 2)
    cv2.line(img, (nx3, ny3), (nx4, ny4), [255, 0, 255], 2)

    # Turn two short lines into one long extended line
    if nx1 != 0:  # two lines found
        k, b = np.polyfit([nx1, nx2, nx3, nx4], [ny1, ny2, ny3, ny4], 1)
    else:  # no line found, continue current line
        return current_line
    # Calculate extended long line's position
    nx1, ny1, nx2, ny2 = int((ysize - b)/k), ysize, int((ysize//1.5 - b)/k), int(ysize/1.5)

    # If new position is not reasonable (out of image)
    if not (0 < nx1 < xsize) or not (0 < nx2 < xsize):
        return current_line

    # If new position is too far from the current ones
    dist_thres = 60
    if current_line != [0, 0, 0, 0] and \
            (abs(nx1 - current_line[0]) > dist_thres or abs(nx2 - current_line[2]) > dist_thres):
        return current_line

    return nx1, ny1, nx2, ny2


def smoother(img, current_line, x1, y1, x2, y2):
    """This function will make the change of line position smoother by
    setting a smoother coefficient."""
    # If new line stays the same as current_line, no need to move
    if current_line[0] == x1 and current_line[2] == x2:
        return current_line

    # Image information
    ysize = img.shape[0]

    # Smoother coefficient
    step = 10
    if current_line != [0, 0, 0, 0]:
        cx1 = current_line[0]
        cx2 = current_line[2]
        x1 = cx1 + (x1 - cx1)/step  # Change in a small step every time
        x2 = cx2 + (x2 - cx2)/step  # Change in a small step every time

    # Fit to find the new gradient k and b in order to draw a new long line
    k, b = np.polyfit([x1, x2], [y1, y2], 1)
    # Return a long line position
    return int((ysize - b)/k), ysize, int((ysize//1.5 - b)/k), int(ysize//1.5)


def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img


# Python 3 has support for cool math symbols.
def weighted_img(img, initial_img, α=0.8, β=1., γ=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    `initial_img` should be the image before any processing.
    The result image is computed as follows:

    initial_img * α + img * β + γ
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, α, img, β, γ)


# Define function to process images
def process_image(image):
    """
    Process a frame of the video
    :param image: Input frame
    :return: Output image with lane indicated
    """
    # Image size
    ysize = image.shape[0]
    xsize = image.shape[1]

    # Gaussian blur and edges detection
    blur_gray = gaussian_blur(image, 11)
    edges = canny(blur_gray, 30, 50)

    # Define interest region
    apex_y = 3/5 * ysize
    vertices = np.array([[(0, ysize), (0.4*xsize, apex_y), (0.6*xsize, apex_y), (xsize, ysize)]], dtype=np.int32)
    masked_edges = region_of_interest(edges, vertices)

    # Hough lines transform
    line_image = hough_lines(masked_edges, 3, np.pi/180, threshold=10, min_line_len=45, max_line_gap=35)

    # Combine found line image and initial image
    result = weighted_img(line_image, image, α=0.8, β=1)

    # Display the result
    # plt.imshow(result)
    # plt.show()

    return result


# Testing with images
test_image_list = os.listdir("test_images/")
# read images one by one
for imageFile in test_image_list:
    # 'current lane' variables are used for smoother lane location change
    # Since testing image, reset this every time
    current_left_lane = [0, 0, 0, 0]
    current_right_lane = [0, 0, 0, 0]
    # Reading image
    img = mpimg.imread("test_images/" + imageFile)
    # Process image
    image_output = process_image(img)
    # Save image
    mpimg.imsave("test_images_output/" + imageFile, image_output)


# Processing videos
current_left_lane = [0, 0, 0, 0]
current_right_lane = [0, 0, 0, 0]
white_output = 'test_videos_output/solidWhiteRight.mp4'
clip1 = VideoFileClip("test_videos/solidWhiteRight.mp4")
white_clip = clip1.fl_image(process_image)
white_clip.write_videofile(white_output, audio=False)

current_left_lane = [0, 0, 0, 0]
current_right_lane = [0, 0, 0, 0]
yellow_output = 'test_videos_output/solidYellowLeft.mp4'
clip2 = VideoFileClip('test_videos/solidYellowLeft.mp4')
yellow_clip = clip2.fl_image(process_image)
yellow_clip.write_videofile(yellow_output, audio=False)

current_left_lane = [0, 0, 0, 0]
current_right_lane = [0, 0, 0, 0]
challenge_output = 'test_videos_output/challenge.mp4'
clip3 = VideoFileClip('test_videos/challenge.mp4')
challenge_clip = clip3.fl_image(process_image)
challenge_clip.write_videofile(challenge_output, audio=False)