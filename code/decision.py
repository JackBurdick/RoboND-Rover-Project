import numpy as np
import collections


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
    Rover.throttle = 0
    Rover.brake = 0

def spin(Rover):
    Rover.throttle = 0
        Rover.brake = Rover.brake_set
    else:
        Rover.brake = 0

def determine_nav_angle(Rover, bias=0):
    degree_buckets = np.round(Rover.nav_angles * 180 / (6 * np.pi)) * 6

    # Filter for only forward angles: [-15 - 15], select most common
    forward_angles = degree_buckets[degree_buckets <= 15.0]
    forward_angles = forward_angles[forward_angles  >= -15]
    angle_options = collections.Counter(forward_angles)
    print(angle_options.most_common(1))
    selected_angle = angle_options.most_common(1)[0][0]
    
    # adjust steer angle for velocity
    if Rover.vel >= 0.9:
        turn_factor = 0.5
    elif Rover.vel >= 0.7:
        turn_factor = 0.6
    else:
        turn_factor = 1

    # bias, used to add bias to steer towards a the left wall (if positive)
    nav_angle = (turn_factor * selected_angle)+bias
    
    return nav_angle


def determine_target_angle(Rover):

    degree_buckets = np.round(Rover.tar_angles * 180 / (6 * np.pi)) * 6

    angle_options = collections.Counter(degree_buckets)
    selected_angle = angle_options.most_common(1)[0][0]
    return selected_angle


# build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # in explore mode, the rover is exploring the map
    if Rover.mode == 'explore':
        print("explore ", end="")
        # first check if we have nav, terrain
        # 5 is a magic number
        if len(Rover.nav_angles) > 5:
            if len(Rover.tar_angles) > 5:
                print("target angles: ", Rover.tar_angles)
                Rover.mode = 'target_ret'
            elif len(Rover.nav_angles) < Rover.stop_forward:
                decelerate(Rover)
                if Rover.vel <= 0.5:
                    # turn right
                    if Rover.steer >= 10 or Rover.steer <= -10:
                        spin(Rover)
                    else:
                        accelerate(Rover)
                    print("no nav, slow, turning right = -15")
                else:
                    print("no nav, moving, new nav = -10")
                    Rover.steer = -10
                   # Rover.steer = determine_nav_angle(Rover)
            else:
                # nav the terrain
                print("naving the terrain")
                # 1 is bias to slightly favor left wall
                Rover.steer = determine_nav_angle(Rover, 0.5)
                accelerate(Rover)
            # if we can see a target, go into target mode


    # in target_ret mode, the rover is getting a target
    elif Rover.mode == 'target_ret':
        print("target_ret ", end="")
        coast(Rover)
        
        # check if near sample and collect
        if Rover.near_sample > 0:
            print("near target, slowing down")
            decelerate(Rover)
            if Rover.vel < 0.2:
                Rover.pick_up = True
                Rover.mode = 'explore'
        
        if len(Rover.tar_angles) <= 8:
            print("in target mode, but not enough targets, back to explore")
            Rover.mode = 'explore'
        
        elif len(Rover.tar_angles) > 8:
            print("can move toward target")
            Rover.steer = determine_target_angle(Rover)
            
            if Rover.steer is 15 or -15:
                spin(Rover)
            else:
                accelerate(Rover)
        

        # else:
        #     print("moving toward target")
        #     accelerate(Rover)


    # Here we can expand on states, for now we'll send the rover back to the
    # exploring state
    else:
        Rover.mode = 'explore'


    # # Check if we have vision data to make decisions with
    # if Rover.nav_angles is not None:
    #     # Check for Rover.mode status
    #     if Rover.mode == 'forward':

    #         if Rover.tar_angles is not None:
    #             # go toward target, there may be obstacles...
    #             if len(Rover.tar_angles) >= Rover.stop_forward:
    #                 if Rover.near_sample > 0:
    #                     # Set mode to "stop" and hit the brakes!
    #                     decelerate(Rover)
    #                     Rover.mode = 'stop'
    #                     if Rover.vel < 0.2:
    #                         Rover.send_pickup = True
    #                 else:
    #                     accelerate(Rover)
    #                     Rover.steer = determine_target_angle(Rover)
                        
    #         # Check the extent of navigable terrain
    #         elif len(Rover.nav_angles) >= Rover.stop_forward:  
    #             accelerate(Rover)
    #             Rover.steer = determine_nav_angle(Rover, 3)
            
    #         # If there's a lack of navigable terrain pixels then go to 'stop' mode
    #         # elif len(Rover.nav_angles) < Rover.stop_forward:
    #         else:
    #                 # Set mode to "stop" and hit the brakes!
    #                 decelerate(Rover)
    #                 Rover.mode = 'stop'

    #     # If we're already in "stop" mode then make different decisions
    #     elif Rover.mode == 'stop':
    #         # If we're in stop mode but still moving keep braking
    #         if Rover.vel > 0.2:
    #             decelerate(Rover)
    #         # Not moving (vel < 0.2) then do something else
    #         elif Rover.vel <= 0.2:
    #             if Rover.near_sample > 0:
    #                 # Set mode to "stop" and hit the brakes!
    #                 decelerate(Rover)
    #                 if Rover.vel < 0.2:
    #                     Rover.send_pickup = True

    #             # Now we're stopped and we have vision data to see if there's a path forward
    #             if len(Rover.nav_angles) < Rover.go_forward:
    #                  # Release the brake to allow turning
    #                  # since we've been favoring left wall, we'll turn right here
    #                 Rover.throttle = 0
    #                 Rover.brake = 0
    #                 Rover.steer = -15
                
    #             # If we're stopped but see sufficient navigable terrain in front then go!
    #             if len(Rover.nav_angles) >= Rover.go_forward:
    #                 # Set throttle back to stored value
    #                 Rover.mode = 'forward'
    #                 accelerate(Rover)
    #                 # Set steer to mean angle
    #                 Rover.steer = determine_nav_angle(Rover, 3)
                    
    # # Just to make the rover do something 
    # # even if no modifications have been made to the code
    # else:
    #     Rover.mode = 'forward'
    #     Rover.brake = 0
    #     Rover.throttle = Rover.throttle_set
    #     accelerate(Rover)
    #     Rover.steer = determine_nav_angle(Rover)

    return Rover

