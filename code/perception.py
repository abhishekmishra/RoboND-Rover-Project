import numpy as np
import cv2

OBSTACLE_COLOR = 100
NAVIGABLE_COLOR = 1
GOLD_COLOR = 200

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    #obstacle_select = np.zeros_like(img[:,:,0])
    #gold_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    
    bgr_img = img[...,::-1]
    # Convert BGR to HSV
    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    
    #print(hsv[150,160])
    
    # define range of gold color in HSV
    lower_gold = np.array([10,100,100])
    upper_gold = np.array([30,255,255])

    # Threshold the HSV image to get only gold colors
    mask = cv2.inRange(hsv, lower_gold, upper_gold)
    
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img, img, mask= mask)
    gold_thresh = (res[:,:,0] > 0) \
                & (res[:,:,1] > 0) 
            
    #print(res[150,160])
    
    #fig = plt.figure(figsize=(12,3))
    #plt.subplot(121)
    #plt.imshow(img)
    #plt.subplot(122)
    #plt.imshow(res)
    
    # Index the array of zeros with the boolean array and set to 1
    color_select[:] = OBSTACLE_COLOR
    color_select[above_thresh] = NAVIGABLE_COLOR
    color_select[gold_thresh] = GOLD_COLOR
    #obstacle_select[above_thresh] = 1
    #obstacle_select[gold_thresh] = 1
    #gold_select[gold_thresh] = 1
    
    # Return the binary image
    return color_select
    #return (color_select, obstacle_select, gold_select)

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped



# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform

    # Define calibration box in source (actual) and destination (desired) coordinates
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6

    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
              [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
              [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
              [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
              ])

    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = (threshed == OBSTACLE_COLOR).astype(np.float) * 255
    Rover.vision_image[:,:,1] = (threshed == GOLD_COLOR).astype(np.float) * 255
    Rover.vision_image[:,:,2] = (threshed == NAVIGABLE_COLOR).astype(np.float) * 255

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    #navigable
    if Rover.pos != None:
        xpos = Rover.pos[0]
        ypos = Rover.pos[1]
       
        yaw = Rover.yaw
        world_size = Rover.worldmap.shape[0]
        scale = 10
        Rover.navpos = []
        Rover.obspos = set([])
    
        margin = 0.6
        roll_max, roll_min = 360-margin, margin
        pitch_max, pitch_min = 360-margin, margin
        
        #navigable
        xpix, ypix = rover_coords(threshed == NAVIGABLE_COLOR)
        x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
        Rover.nav_x_w, Rover.nav_y_w = x_world, y_world
        if (Rover.pitch < pitch_min or Rover.pitch > pitch_max) and (Rover.roll < roll_min or Rover.roll > roll_max):
            Rover.worldmap[y_world, x_world, 2] += 1
        for i in range(len(xpix)):
            pos = (x_world[i], y_world[i])
            Rover.navpos.append(pos)
        Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix, ypix)
        
        #obstacle
        xpix, ypix = rover_coords(threshed == OBSTACLE_COLOR)
        x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
        for i in range(len(x_world)):
            pos = (x_world[i], y_world[i])
            Rover.obspos.add(pos)
        if (Rover.pitch < pitch_min or Rover.pitch > pitch_max) and (Rover.roll < roll_min or Rover.roll > roll_max):
            Rover.worldmap[y_world, x_world, 0] += 1
    
        #gold
        xpix, ypix = rover_coords(threshed == GOLD_COLOR)
        x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
        if (Rover.pitch < pitch_min or Rover.pitch > pitch_max) and (Rover.roll < roll_min or Rover.roll > roll_max):
            Rover.worldmap[y_world, x_world, 1] += 1
        Rover.gold_dists, Rover.gold_angles = to_polar_coords(xpix, ypix)
    
        # 8) Convert rover-centric pixel positions to polar coordinates
        # Update Rover pixel distances and angles
            # Rover.nav_dists = rover_centric_pixel_distances
            # Rover.nav_angles = rover_centric_angles
        
        Rover.threshed = threshed

    return Rover
