import numpy as np
import collections
import time


def accelerate(Rover):
    Rover.brake = 0
    if Rover.vel < Rover.max_vel:
        # accelerate
        Rover.throttle = Rover.throttle_set
    else:
        # coast
        Rover.throttle = 0

def decelerate(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0

def coast(Rover):
    Rover.throttle = 0
    Rover.brake = 0

def spin(Rover):
    Rover.throttle = 0
    if Rover.vel >= 0.2:
        Rover.brake = Rover.brake_set
    else:
        Rover.brake = 0

def stop(Rover):
    Rover.steer = 0
    Rover.throttle = 0
    Rover.brake = Rover.brake_set

def determine_nav_angle(Rover, bias=0):

    close_obs = []
    i = 0
    for j in Rover.obs_dists:
        if j <= 30:
            close_obs.append(Rover.obs_angles[i])
    
    far_navs = []
    m = 0
    for n in Rover.nav_dists:
        if n >= 20:
            far_navs.append(Rover.nav_angles[i])


    f_drive_angles = [x for x in far_navs if x not in close_obs]
    f_drive_angles = np.asarray(f_drive_angles)
    # print(f_drive_angles)
    f_drive_buckets = np.round(f_drive_angles * 180 / (7 * np.pi)) * 7
    f_drive_angles = f_drive_buckets[f_drive_buckets <= 15.0]
    f_drive_angles = f_drive_angles[f_drive_angles >= -15]
    
    # There may not be any options to move forward; if so, turn right
    if len(f_drive_angles) == 0:
        nav_angle = -15
    else:
        angle_options = collections.Counter(f_drive_angles)
        # print(angle_options.most_common(1))
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

    degree_buckets = np.round(Rover.tar_angles * 180 / (5 * np.pi)) * 5

    angle_options = collections.Counter(degree_buckets)
    selected_angle = angle_options.most_common(1)[0][0]
    return selected_angle


# build a decision tree for determining throttle, brake and steer 
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

        # first check if we have nav, terrain
        # 5 is a magic number
        elif len(Rover.nav_angles) > 5:
            if len(Rover.tar_angles) > 5:
                print("target angles: ", Rover.tar_angles)
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
                            Rover.steer = -8
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
                            Rover.steer = -14
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
                    Rover.steer = -8
                    spin(Rover)
        # if we can see a target, go into target mode
        else:
            # turn right
            if len(Rover.tar_angles) > 5:
                print("target angles: ", Rover.tar_angles)
                Rover.mode = 'target_ret'
            else:
                print("nothing in front, spining")
                Rover.steer = -8
                spin(Rover)


    # in target_ret mode, the rover is getting a target
    elif Rover.mode == 'target_ret':
        print("target_ret ", end="")
        print("dist to target = ", Rover.tar_dists, end=" ")
        coast(Rover)
        
        # check if near sample and collect
        print(Rover.near_sample)
        if Rover.near_sample:
            print("near target, slowing down")
            stop(Rover)
            if Rover.vel < 0.2:
                Rover.send_pickup = True
    
        
        if len(Rover.tar_angles) <= 8:
            print("in target mode, but not enough targets, back to explore")
            Rover.mode = 'explore'
        
        elif len(Rover.tar_angles) > 8:
            print("can move toward target")
            Rover.steer = determine_target_angle(Rover)
            if np.mean(Rover.tar_dists) <= 50:
                coast(Rover)
            else:
                accelerate(Rover)
            
            # if Rover.steer >= 15 or Rover.steer <= -15:
            #     spin(Rover)
            # else:
            #     accelerate(Rover)
        

        # else:
        #     print("moving toward target")
        #     accelerate(Rover)


    # Here we can expand on states, for now we'll send the rover back to the
    # exploring state
    else:
        Rover.mode = 'explore'

    return Rover

