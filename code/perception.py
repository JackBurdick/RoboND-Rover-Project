import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160), nav=True):
    # Create an array of zeros same xy size as img, but single channel
    if nav:
        color_select = np.zeros_like(img[:,:,0])
    else:
        color_select = np.ones_like(img[:,:,0])
    
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
   
    # Index the array of zeros with the boolean array and set to 1
    if nav:
        color_select[above_thresh] = 1
    else:
        color_select[above_thresh] = 0
    # Return the binary image
    return color_select

def color_thresh_gold(img, hsv_thresh_lower=(10, 150, 100), hsv_thresh_upper=(255, 255, 255)):
    color_select = np.zeros_like(img[:,:,0])

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #modify the upper and lower bounds of the filter
    #to alter the filter Tektron3000. BGR
    lower_gold = np.array([hsv_thresh_lower[0], hsv_thresh_lower[1], hsv_thresh_lower[2]])
    upper_gold = np.array([hsv_thresh_upper[0], hsv_thresh_upper[1], hsv_thresh_upper[2]])

    mask = cv2.inRange(hsv, lower_gold, upper_gold)

    # convert to binary
    # TODO: rewrite - https://stackoverflow.com/questions/41672268/convert-numpy-array-with-floats-to-binary-0-or-1-integers
    i = 0
    for col in mask:
        j = 0
        for val in col:
            if val > 100:
                #print(val)
                color_select[i][j] = 1
            else:
                color_select[i][j] = 0
            j += 1
        i += 1

    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
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

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):

    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
 
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale))

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
    # NOTE: camera image is coming to you in Rover.img
    img = Rover.img
    data_Xpos = Rover.pos[0]
    data_Ypos = Rover.pos[1]
    data_Yaw = Rover.yaw
    
    # 1) Define source and destination points for perspective transform
    dst_size = 5

    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    bottom_offset = 5
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])

    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    rgb_thresh=(160, 160, 160)
    bin_img = color_thresh(warped, rgb_thresh)
    bin_img_obs = color_thresh(warped, rgb_thresh, nav=False)
    bin_img_tar = color_thresh_gold(warped)


    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = bin_img_obs
    Rover.vision_image[:,:,1] = bin_img_tar
    Rover.vision_image[:,:,2] = bin_img

    # 5) Convert map image pixel values to rover-centric coords
    xpix_nav, ypix_nav = rover_coords(bin_img)
    xpix_obs, ypix_obs = rover_coords(bin_img_obs)
    xpix_tar, ypix_tar = rover_coords(bin_img_tar)

    scale = 10
    
    dist_nav, angles_nav = to_polar_coords(xpix_nav, ypix_nav)
    mean_dir_nav = np.mean(angles_nav)
    
    dist_obs, angles_obs = to_polar_coords(xpix_obs, ypix_obs)
    mean_dir_obs = np.mean(angles_obs)
    
    dist_tar, angles_tar = to_polar_coords(xpix_tar, ypix_tar)
    mean_dir_tar = np.mean(angles_tar)

    # 6) Convert rover-centric pixel values to world coordinates
    # obstacle
    obs_x_world, obs_y_world = pix_to_world(xpix_obs, ypix_obs, 
                                            data_Xpos, 
                                            data_Ypos, 
                                            data_Yaw, 
                                            Rover.worldmap.shape[0], 
                                            scale)
    # target (gold)
    tar_x_world, tar_y_world = pix_to_world(xpix_tar, ypix_tar, 
                                            data_Xpos, 
                                            data_Ypos, 
                                            data_Yaw, 
                                            Rover.worldmap.shape[0], 
                                            scale)
    # navigable
    navigable_x_world, navigable_y_world = pix_to_world(xpix_nav, ypix_nav, 
                                                        data_Xpos, 
                                                        data_Ypos, 
                                                        data_Yaw, 
                                                        Rover.worldmap.shape[0], 
                                                        scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
    Rover.worldmap[tar_y_world, tar_x_world, 1] += 1
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    dist_nav, angles_nav = to_polar_coords(xpix_nav, ypix_nav)
    #mean_dir_nav = np.mean(angles_nav)
    
    dist_obs, angles_obs = to_polar_coords(xpix_obs, ypix_obs)
    #mean_dir_obs = np.mean(angles_obs)
    
    dist_tar, angles_tar = to_polar_coords(xpix_tar, ypix_tar)
    #mean_dir_tar = np.mean(angles_tar)
    
    Rover.nav_angles = angles_nav
    Rover.nav_dists = dist_nav
    Rover.obs_angles = angles_obs
    Rover.obs_dists = dist_obs
    Rover.tar_angles = angles_tar
    Rover.tar_dists = dist_tar
 
    return Rover