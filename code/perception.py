import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh_min=(160, 160, 160), rgb_thresh_max=(255, 255, 255)): # 215, 205, 180
    # Create an array of zeros same xy size as img, but single channel
    img_mask = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met.
    identified_pixels = (img[:, :, 0] > rgb_thresh_min[0]) \
                & (img[:, :, 1] > rgb_thresh_min[1]) \
                & (img[:, :, 2] > rgb_thresh_min[2]) \
                \
                & (img[:, :, 0] <= rgb_thresh_max[0]) \
                & (img[:, :, 1] <= rgb_thresh_max[1]) \
                & (img[:, :, 2] <= rgb_thresh_max[2])
    
    # Index the array of zeros with the boolean array and set to 1
    img_mask[identified_pixels] = 1
    
    # Return the binary image
    return img_mask

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels.
    ypos, xpos = binary_img.nonzero()
    
    # Calculate pixel positions with reference to the rover position being at the center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(float)
    y_pixel = -(xpos - binary_img.shape[1]/2).astype(float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space.
# I.e., convert (x_pixel, y_pixel) to (distance, angle) in polar coordinates in rover space.
def to_polar_coords(x_pixel, y_pixel):
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians.
    yaw_rad = yaw * np.pi / 180
    
    # Apply rotation by multiplying the image by the 2D rotation matrix.
    # Rotation matrix: [x`, y`] = [cos(x) -sin(y); sin(x) cos(y)][x; y]
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply translation with scaling.
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    
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
    
    return x_pix_world, y_pix_world

def perspect_transform(img, src, dst):
    # Get a transformation matrix from the corners of the desired image section and a matrix containting the new image corner pixels.
    trans_matrix = cv2.getPerspectiveTransform(src, dst)
    
    # Inputs are: [The original image, transformation matrix, the (height (y), width (x)) of the output image]
    # Since we want to keep same size of the input image, we need to pass its shape.
    return cv2.warpPerspective(img, trans_matrix, (img.shape[1], img.shape[0]))


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Make sure to not update the map at the start of the simulation to prevent wrong values.
    # if s:=all([abs(abs(Rover.pos[0]) - 99.7) <= 1, abs(abs(Rover.pos[1]) - 85.6) <= 1]) and (Rover.samples_collected in [0, 6]):
    #     print(f"Near the starting position of the simulation {s}. No mapping is done.")
    #     return Rover
    
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img
    image = Rover.img
    
    ## 1) Define source and destination points for perspective transform
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 10
    
    # The pixel coordinates below are found by opening `example_grid1.jpg` with paint and hovering over the grid corners
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]]) # (B)ottom (L)eft, BR, TR, TL
    
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],    # Y-size center - dst_size, X-size - btm_offset
                    [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],              # Y-size center + dst_size, X-size - btm_offset
                    [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], # Y-size center + dst_size, X-size - 2*dst_size - btm_offset
                    [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset], # Y-size center - dst_size, X-size - 2*dst_size - btm_offset
                    ])
    
    ## 2) Apply perspective transform
    warped = perspect_transform(image, source, destination)
    
    ## 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    nav_terrain_threshed = color_thresh(warped, rgb_thresh_min=(190, 180, 165), rgb_thresh_max=(255, 255, 230))
    obs_threshed   = color_thresh(warped, rgb_thresh_min=(0, 0, 0), rgb_thresh_max=(160, 160, 160))
    
    # Rocks are bright in red and green channels, and dim in blue channel
    # rock_threshed  = color_thresh(warped, rgb_thresh_min=(150, 100, 0), rgb_thresh_max=(255, 200, 80))
    rock_threshed  = color_thresh(warped, rgb_thresh_min=(140, 115, 0), rgb_thresh_max=(255, 200, 80))
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:, :, 0] = obs_threshed*255
    Rover.vision_image[:, :, 1] = rock_threshed*255
    Rover.vision_image[:, :, 2] = nav_terrain_threshed*255
    
    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(nav_terrain_threshed)
    obs_xpix, obs_ypix = rover_coords(obs_threshed)
    rock_xpix, rock_ypix = rover_coords(rock_threshed)
    
    # 6) Convert rover-centric pixel values to world coordinates
    xpos, ypos, yaw = Rover.pos[0], Rover.pos[1], Rover.yaw
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, xpos, ypos, yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, xpos, ypos, yaw, world_size, scale)
    
    
    # 7) Convert rover-centric pixel positions to polar coordinates
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix, ypix)
    Rover.rock_dist, Rover.rock_angles = to_polar_coords(rock_xpix, rock_ypix)
    
    
    # 8) Update Rover worldmap (to be displayed on right side of screen)
    # Update world map if we are not turning around or tilted more than 5 degrees to ensure good precision.
    # Roll angle can be described as the rotation of an object around its longitudinal axis (side-to-side).
    # Pitch angle can be described as the rotation (flipping) of an object due to acceleration (front-to-rear).
    if Rover.starting_counter <= 120:
        print(f"Starting the simulation. Delayed mapping: no mapping is done. Frames until the end of the delay: {Rover.starting_counter}/120\r", end="")
        Rover.starting_counter += 1
        Rover.throttle_set = 0.3
        if Rover.starting_counter == 121:
            Rover.throttle_set = 0.7
            print("\n")
    
    elif ((0 <= Rover.roll  < 2) or (360 >= Rover.roll > 358)) and \
        ((0 <= Rover.pitch <= 1) or (360 >= Rover.pitch >= 359)) and \
            not Rover.send_pickup and \
            not Rover.brake and \
            Rover.vel >= 0 and \
            not (Rover.throttle >= 0.2 and Rover.vel == 0) and \
            not (abs(Rover.steer) == 15 and Rover.vel == 0): # len(Rover.nav_angles) <= 4500 and 
        # not (Rover.rock_detected and (Rover.roll > 2 or Rover.roll)) 
        # Check for pixels with value > 255 => Rover.worldmap[Rover.worldmap[: , :, 0] > 255, 0] = 255
        
        # Reset the red channel where there are blue pixels.
        Rover.worldmap[Rover.worldmap[:, :, 2] > 160, 0] = 0
        if Rover.mode != 'stuck':
            # Update blue channel where there is navigable terrain
            Rover.worldmap[y_world, x_world, 2] += 7
            
            # make sure all blue pixels don't exceed 255:
            Rover.worldmap[Rover.worldmap[:, :, 2] > 255, 2] = 255
    
    # Update red channel where there are obstacles.
    Rover.worldmap[obs_y_world, obs_x_world, 0] += 2
    
    # Update green channel where there are rocks.
    Rover.worldmap[rock_y_world, rock_x_world, 1] = 255
    
    
    # Clear out low certainty navigable terrain pixels every 100 frames to increase fidelity.
    if Rover.iteration_counter % 100 == 0:
        # Find navigable terrain pixels
        nav_terrain_pixels = Rover.worldmap[:, :, 2] > 0
        
        # Define low certainty pixel as having a value less than one-fourth of the average.
        # Set low quality pixels to zero.
        if nav_terrain_pixels.any():
            low_certainty_pixel_value = np.mean(Rover.worldmap[nav_terrain_pixels, 2]) / 4
            low_certainty_pixels = Rover.worldmap[:, :, 2] < max(low_certainty_pixel_value, 100)
            Rover.worldmap[low_certainty_pixels, 2] = 0
            
            print("\nMean blue pixles value:", int(low_certainty_pixel_value*4))
            
            # # Do the opposite of the above; set the hight certainty navigable terrain pixels to 255
            # high_certainty_pixels = Rover.worldmap[:, :, 2] > 50
            # Rover.worldmap[high_certainty_pixels, 2] = 255
            
        # Reset the counter
        Rover.iteration_counter = 0
    
    # # If a rock is encountered, update direction towards it.
    # if len(Rover.rock_dist) > 0:
    #     Rover.nav_dists = Rover.rock_dist
    #     Rover.nav_angles = Rover.rock_angles
    # else: # Update the direction towards navigable terrain.
    #     if Rover.mode == 'rock_visible':
    #         Rover.mode = 'forward'
    
    # Rover.nav_dists = rock_dist
    # Rover.nav_angles = rock_angles
    
    Rover.iteration_counter += 1
    return Rover
