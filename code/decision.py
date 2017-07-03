import numpy as np
import collections
import time

############################################### Aux functions

def accelerate(Rover):
    ''' accelerate will increase the rovers speed.
    If the rover is already at max velocity, the rover will only coast
    '''
    Rover.brake = 0
    if Rover.vel < Rover.max_vel:
        # accelerate
        Rover.throttle = Rover.throttle_set
    else:
        # coast
        Rover.throttle = 0


def decelerate(Rover):
    ''' declerate will slow down the rover. '''
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0


def coast(Rover):
    ''' coast will allow the rover to coast '''
    Rover.throttle = 0
    Rover.brake = 0


def spin(Rover):
    ''' spin will slow the rover then allow the rover to spin'''
    Rover.throttle = 0
    if Rover.vel >= 0.2:
        Rover.brake = Rover.brake_set
    else:
        Rover.brake = 0


def stop(Rover):
    ''' stop will cause the rover to stop in its current location '''
    Rover.steer = 0
    Rover.throttle = 0
    Rover.brake = Rover.brake_set


def determine_spin_dir(Rover):
    ''' determine_spin_dir will determine if the rover should spin left or right
    The rover will count the number of navigable pixels on both sides of the
    rover and choose the direction towards the most pixels  '''
    nav_ok = np.round(Rover.nav_angles * 180 / (2 * np.pi)) * 2
    right_count = nav_ok[nav_ok < 0]
    left_count = nav_ok[nav_ok > 0]
    if len(right_count) >= len(left_count):
        nav_angle = -15
    elif len(left_count) > len(right_count):
        nav_angle = 15
    else:
        # default to right
        nav_angle = -15
    return nav_angle


def determine_nav_angle(Rover, bias=0):
    ''' determine_nav_angle will determine which direction the rover should
    steer.  The nav angels that are drivable over a certain distance will be 
    first found, then the angels that include obstacles will be subtracted from
    all these angles.  A `turn_factor` is used to prevent the rover from
    attempting a full turn at max speed. '''

    close_obs = []
    i = 0
    for j in Rover.obs_dists:
        if j <= 30:
            close_obs.append(Rover.obs_angles[i])
    
    far_navs = []
    m = 0
    for n in Rover.nav_dists:
        if n >= 10:
            far_navs.append(Rover.nav_angles[i])


    f_drive_angles = [x for x in far_navs if x not in close_obs]
    f_drive_angles = np.asarray(f_drive_angles)
    f_drive_buckets = np.round(f_drive_angles * 180 / (5 * np.pi)) * 5
    f_drive_angles = f_drive_buckets[f_drive_buckets <= 15.0]
    f_drive_angles = f_drive_angles[f_drive_angles >= -15]
    
    # There may not be any options to move forward; if so, turn right
    if len(f_drive_angles) == 0:
        selected_angle = determine_spin_dir(Rover)

    else:
        angle_options = collections.Counter(f_drive_angles)
        selected_angle = angle_options.most_common(1)[0][0]

    # adjust steer angle for velocity
    if Rover.vel >= 0.9:
        turn_factor = 0.8
    elif Rover.vel >= 0.7:
        turn_factor = 0.9
    else:
        turn_factor = 1

    # bias, used to add bias to steer towards a the left wall (if positive)
    nav_angle = (turn_factor * selected_angle)+bias
    
    return nav_angle


def determine_target_angle(Rover):
    ''' determine_target_angle determines which direction the rover should steer
    to acquire a rock '''

    degree_buckets = np.round(Rover.tar_angles * 180 / (5 * np.pi)) * 5

    angle_options = collections.Counter(degree_buckets)
    selected_angle = angle_options.most_common(1)[0][0]
    return selected_angle


############################################### Main DT
# Build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # in explore mode, the rover is exploring the map
    if Rover.mode == 'explore':
        print("explore ", end="")

        # if near a target, get sample
        if Rover.near_sample:
            print("near target, slowing down")
            stop(Rover)
            if Rover.vel < 0.2:
                Rover.send_pickup = True

        # first check if we have nav. terrain        
        elif len(Rover.nav_angles) > 5:
            if len(Rover.tar_angles) > 3:
                Rover.mode = 'target_ret'

            elif len(Rover.nav_angles) < Rover.stop_forward:
                decelerate(Rover)
                if Rover.vel <= 0.5:
                   
                    if Rover.steer >= 10 or Rover.steer <= -10:
                        spin(Rover)
                    else:
                        accelerate(Rover)
                        # if we're "stuck"
                        if Rover.throttle > 0 and (Rover.vel <= 0.05 and Rover.vel >= -0.05):
                            Rover.throttle = 0
                            Rover.steer = determine_spin_dir(Rover)
                    print("no nav, slow, turning right = -8")
                else:
                    print("no nav, moving, new nav = -8")
                    Rover.steer = -8
                   # Rover.steer = determine_nav_angle(Rover)
            
            else:
                if len(Rover.nav_angles) > 5:
                    # nav the terrain
                    print("naving the terrain")
                    
                    # Steering
                    # block for being "stuck"
                    if Rover.throttle > 0 and (Rover.vel <= 0.05 and Rover.vel >= -0.05):
                        if Rover.stuck_count == 60:
                            Rover.throttle = 0
                            Rover.steer = determine_spin_dir(Rover)
                            Rover.stuck_count = 0
                            print("stuck count limit reached:", Rover.stuck_count)
                        else:
                            Rover.stuck_count += 1
                    
                    # normal navigation
                    else:
                        print("normal nav")
                        Rover.steer = determine_nav_angle(Rover)
                    
                    # Forward motion
                    if Rover.steer >= 14 or Rover.steer <= -14:
                        if Rover.vel <= 0.05:
                            print("spin")
                            spin(Rover)
                        else:
                            print("accel")
                            accelerate(Rover)
                    else:
                        print("f_accel")
                        accelerate(Rover)
                else:
                    print("nothing in front, spining")
                    Rover.steer = determine_spin_dir(Rover)
                    spin(Rover)
        
        # if we can see a target, go into target mode
        else:
            if len(Rover.tar_angles) > 3:
                Rover.mode = 'target_ret'
            else:
                print("nothing in front, spining")
                Rover.steer = determine_spin_dir(Rover)
                spin(Rover)


    # in target_ret mode, the rover is getting a target
    elif Rover.mode == 'target_ret':
        print("target_ret ", end="")
        coast(Rover)
        
        # check if near sample and collect
        print(Rover.near_sample)
        if Rover.near_sample:
            print("near target, slowing down")
            stop(Rover)
            if Rover.vel < 0.2:
                Rover.send_pickup = True
    
        
        if len(Rover.tar_angles) <= 2:
            print("in target mode, but not enough targets, back to explore")
            Rover.mode = 'explore'
        
        elif len(Rover.tar_angles) > 2:
            print("can move toward target")
            Rover.steer = determine_target_angle(Rover)

            if np.mean(Rover.tar_dists) < 100:
                if Rover.vel >= 1:
                    decelerate(Rover)
                if Rover.vel >= 0.5:
                    coast(Rover)
                else:
                    accelerate(Rover)


    # Here we could expand on states. For now, we'll send the rover back to the
    # exploring state
    else:
        Rover.mode = 'explore'

    return Rover

