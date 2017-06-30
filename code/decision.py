import numpy as np


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


def determine_steer_angle(Rover):
    # 3 is a magic number, used to add bias to steer towards a the left wall
    steer_angle = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -7, 7)+8
    return steer_angle


def determine_target_angle(Rover):
    steer_angle = np.clip(np.mean(Rover.tar_angles * 180/np.pi), -15, 15)
    return steer_angle


# build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':

            if Rover.tar_angles is not None:
                # go toward target, there may be obstacles...
                if len(Rover.tar_angles) >= Rover.stop_forward:
                    if Rover.near_sample > 0:
                        # Set mode to "stop" and hit the brakes!
                        Rover.throttle = 0
                        # Set brake to stored brake value
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        Rover.mode = 'stop'
                        Rover.send_pickup = True
                    else:
                        accelerate(Rover)
                        Rover.steer = determine_target_angle(Rover)
            
            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                accelerate(Rover)
                Rover.steer = determine_steer_angle(Rover)
            
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                decelerate(Rover)
            # Not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                if Rover.near_sample > 0:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.send_pickup = True

                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                     # Release the brake to allow turning
                     # since we've been favoring left wall, we'll turn right here
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15
                
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.mode = 'forward'
                    accelerate(Rover)
                    # Set steer to mean angle
                    Rover.steer = determine_steer_angle(Rover)
                    
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.brake = 0
        accelerate(Rover)
        Rover.steer = determine_steer_angle(Rover)

    return Rover

