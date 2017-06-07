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


MIN_DIST_TO_MOVE = 1

#min x which is unblocked
def get_min_x_unblocked(mp):
    for i in range(199, -1, -1):
        for j in range(199, -1, -1):
            if mp[i, j] > 0.0:
                return i, j
#lowest_ub = get_min_x_unblocked(mp)
#print(lowest_ub)

def is_on_border(mp, pos, r=4):
    for di in range(-r, r+1):
        for dj in range(-r, r+1):
            if mp[pos[0] + di, pos[1] + dj] == 0.0:
                return True
    return False
#is_on_border(mp, lowest_ub)

DIST_FROM_WALL = 2

def all_neighbors_green(mp, pos):
    green = True
    for i in range(-1, 2):
        for j in range(-1, 2):
            if mp[(pos[0]+i, pos[1]+j)] != 1.0:
                green = False
    return green

def median_path(mp, left_right = True):
    lmeans = []
    rmeans = []
    for i in range(mp.shape[0]):
        start = None
        end = None
        inside = False
        for j in range(mp.shape[1]):
            if left_right == True:
                pos = (i, j)
            else:
                pos = (j, i)
            if mp[pos] == 1.0 and not inside:
                inside = True
                start = j
            if mp[pos] == 0.0 and inside:
                inside = False
                end = j
            if start != None and end != None:
                #wh = start + DIST_FROM_WALL
                wh = int((start + end)/2)
                if left_right == True:
                    new_pos = (i, wh)
                else:
                    new_pos = (wh, i)
                if all_neighbors_green(mp, new_pos):
                    lmeans.append(new_pos)
                
#                wh = end - DIST_FROM_WALL
#                if left_right == True:
#                    new_pos = (i, wh)
#                else:
#                    new_pos = (wh, i)
#                if mp[new_pos] == 1.0:
#                    rmeans.append(new_pos)
                
                start = None
                end = None
    return lmeans + rmeans

def distance(pos1, pos2):
    return np.sqrt(np.square(pos1[0] - pos2[0]) + np.square(pos1[1] - pos2[1]))

def sign_changes(start, end, new_pos):
    sc = 0
    if np.sign(new_pos[0] - end[0]) != np.sign(end[0] - start[0]):
        sc += 1
    if np.sign(new_pos[1] - end[1]) != np.sign(end[1] - start[1]):
        sc += 1
    return sc

def find_min_not_visited(prev, pos, not_visited):
    min_dist = None
    min_pos = None
    min_sc = None
    for sc in range(3):
        for to in not_visited:
            dc = sign_changes(prev, pos, to)
            dist = distance(pos, to)
            if min_dist == None or (dc == sc and dist < min_dist and dist > MIN_DIST_TO_MOVE):
                min_dist = dist
                min_pos = to
                min_sc = dc
        if min_dist != None and min_dist < 20:
            break
    if min_dist != None and min_dist < 50:
        #print(min_dist, prev, pos, min_pos, min_sc)
        return min_pos
    else:
        return None

def greedy_path(current_path):
    #visited = set([])
    new_path = []
    not_visited = set(current_path)
    current = current_path[0]
    prev = current
    while current != None and current in not_visited:
        not_visited.discard(current)
        new_path.append(current)
        prev = current
        current = find_min_not_visited(prev, current, not_visited)
    print('not visited', len(not_visited))
    return new_path


def closest_point_on_path(Rover, visited = False):
    min_dist = None
    min_pos = None
    #print(Rover.not_visited)
    path = None
    if visited:
        path = Rover.apath
    else:
        path = Rover.not_visited
    
    for to in path:
        #rover_pt = world_to_pix(to[0], to[1], Rover.pos[0], Rover.pos[1], Rover.yaw)
        #d = int(np.sqrt(rover_pt[0]**2 + rover_pt[1]**2))
        #a = int(np.arctan2(rover_pt[1], rover_pt[0]) * 180/np.pi)
        #if coord_navigable(to[0], to[1], Rover):
        dist = distance(Rover.pos, to)
        if min_dist == None or dist < min_dist:
            min_dist = dist
            min_pos = to
    print(min_dist, min_pos)
    return min_pos

def coord_navigable(x, y, Rover):
    for i in range(len(Rover.nav_x_w)):
        if x == Rover.nav_x_w[i] and y == Rover.nav_y_w[i]:
            return True
    return False

def get_next_points(Rover, next_pos, num = 2):
    next_points_world = np.zeros((num, 2))
    next_points_rover = np.zeros((num, 2))
    next_points_rover_polar = []
    for i in range(len(Rover.apath)):
        if next_pos == Rover.apath[i]:
            for j in range(num):
                if i+j < len(Rover.apath):
                    pt = Rover.apath[i+j]
                    next_points_world[j] = pt
                    rover_pt = world_to_pix(pt[0], pt[1], Rover.pos[0], Rover.pos[1], Rover.yaw)
                    next_points_rover[j] = rover_pt
                    dist = np.sqrt(rover_pt[0]**2 + rover_pt[1]**2)
                    angle = np.arctan2(rover_pt[1], rover_pt[0]) * 180/np.pi
                    #if coord_navigable(dist, angle, Rover.int_dists, Rover.int_angles):
                    next_points_rover_polar.append((dist, angle))
    #print (next_points_world, next_points_rover)
    return next_points_rover_polar

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def world_to_pix(xw, yw, xpos, ypos, yaw, rover_size=2000, scale=0.1):
    xwo = -xpos/scale
    ywo = -ypos/scale
    xwo, ywo = rotate_pix(xwo, ywo, -yaw)
    #print('world origin', xwo, ywo)
    # Apply rotation
    xw_rot, yw_rot = rotate_pix(xw, yw, -yaw)
    #print(xw_rot, yw_rot)
    # Apply translation
    xw_tran = xwo + xw_rot/scale
    yw_tran = ywo + yw_rot/scale
    #xw_tran, yw_tran = translate_pix(xw_rot, yw_rot, xwo, ywo, scale)
    #print(xw_tran, yw_tran)
    
    # Perform rotation, translation and clipping all at once
    x_pix = np.clip(np.int_(xw_tran), 0, rover_size - 1)
    y_pix = np.clip(np.int_(yw_tran), 0, rover_size - 1)
    # Return the result
    return int(xw_tran), int(yw_tran)


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
        print(Rover.pos)
        xpos = Rover.pos[0]
        ypos = Rover.pos[1]
       
        yaw = Rover.yaw
        world_size = Rover.worldmap.shape[0]
        scale = 10
        Rover.navpos = []
        Rover.obspos = set([])
    
        #navigable
        xpix, ypix = rover_coords(threshed == NAVIGABLE_COLOR)
        x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
        Rover.nav_x_w, Rover.nav_y_w = x_world, y_world
        if (Rover.pitch < 1 or Rover.pitch > 359) and (Rover.roll < 1 or Rover.roll > 359):
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
        if (Rover.pitch < 1 or Rover.pitch > 359) and (Rover.roll < 1 or Rover.roll > 359):
            Rover.worldmap[y_world, x_world, 0] += 1
    
        #gold
        xpix, ypix = rover_coords(threshed == GOLD_COLOR)
        x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
        if (Rover.pitch < 1 or Rover.pitch > 359) and (Rover.roll < 1 or Rover.roll > 359):
            Rover.worldmap[y_world, x_world, 1] += 1
        Rover.gold_dists, Rover.gold_angles = to_polar_coords(xpix, ypix)
    
        # 8) Convert rover-centric pixel positions to polar coordinates
        # Update Rover pixel distances and angles
            # Rover.nav_dists = rover_centric_pixel_distances
            # Rover.nav_angles = rover_centric_angles
        
        Rover.threshed = threshed
        
    #    if Rover.medians == None:
    #        lrmedians = median_path(Rover.mp)
    #        upmedians = median_path(Rover.mp, False)
    #        Rover.medians = lrmedians + upmedians
    #        Rover.apath = greedy_path(Rover.medians)
    #        print ('rover path', Rover.apath)
    #        for p in Rover.apath:
    #            mark_neighbors(Rover, p[0], p[1])
    #        Rover.not_visited = set(Rover.apath)
    #        Rover.visited = set([])
    #        Rover.int_dists = Rover.nav_dists.astype(int)
    #        Rover.int_angles = Rover.nav_angles_deg().astype(int)
    #    
    #    Rover.nav_unique_pos = set([])
    #    for i in range(len(Rover.nav_x_w)):
    #        pos = (Rover.nav_x_w[i], Rover.nav_y_w[i])
    #        Rover.nav_unique_pos.add(pos)
        #print('unique world positions navigable', Rover.nav_unique_pos)

    return Rover

def mark_neighbors(Rover, x, y):
    for i in range(-1, 2):
        for j in range(-1, 2):
            Rover.worldmap[x, y, 0] = 220
            Rover.worldmap[x, y, 1] = 220
            Rover.worldmap[x, y, 2] = 220