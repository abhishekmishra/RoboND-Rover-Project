import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        print('can go forward', canGoForward(Rover), 'steering angle', getSteerAngle(Rover))
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if canGoForward(Rover):  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                Rover.steer = getSteerAngle(Rover)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            else:
                print('##################### ROVER STOPPING #####################')
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
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = getSteerAngle(Rover)
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                if canGoForward(Rover):
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.steer = getSteerAngle(Rover)
                    Rover.mode = 'forward'
                # Now we're stopped and we have vision data to see if there's a path forward
                else:
                    print('##################### ROVER TURNING #####################')
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        print('##################### ROVER STOPPED #####################')
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover


def getSteerAngle(Rover):
    '''
    At any position of the rover, we compute the steer angle
    based on navigation angles and distances.
    
    1. if the angles are the same sign we assume there's an obstacle in front
       and we stop and get away
    2. to hug the left wall for a clockwise path
        a. we try to reach a place where average navigable distance on
           the left is less than the navigable distance on the right
        b. and then we maintain this condition
    '''
    
    angles = Rover.nav_angles_rad()
    steer_angle = None
    
    if len(angles) > 0:
        max_angle = np.max(angles)
        min_angle = np.min(angles)
        
        mean_dist = np.mean(Rover.nav_dists)
        print ('Mean navigable distance =', mean_dist)
    
        left_angle = max_angle
        right_angle = np.abs(min_angle)
        
        print(min_angle, max_angle, 'left available = ', 
              left_angle, 'right available', right_angle)

        #angles are same sign, means obstacle in front, get away
        if (max_angle <= 0 and min_angle <= 0):
            print('both angles are negative')
            steer_angle = min_angle
        elif (max_angle > 0 and min_angle > 0):
            print('both angles are positive')
            steer_angle = max_angle
        else:    
            #angle_range = max_angle - min_angle
            if left_angle > right_angle:
                steer_angle = left_angle
                print('left has more space - turning left', steer_angle)
            elif left_angle < 10:
                steer_angle = min_angle
                print('too close to facing left wall - turning right', steer_angle)
            else:
                steer_angle = np.mean(angles) + max_angle/5
                print('keep sort of left/straight', steer_angle)

    if steer_angle != None:
        steer_angle = np.clip(steer_angle, -15, 15)
        
    return steer_angle

def isNavigable(Rover, angle):
    int_angles = Rover.nav_angles_rad().astype(int)
    #print (int_angles, angle)
    all_present = True
    for a in range(int(angle) - 3, int(angle) + 3):
        if not a in int_angles:
            all_present = False
            break
    return all_present

#def alreadySeen(Rover, angle):

def canGoForward(Rover):
    steerAngle = getSteerAngle(Rover)
    if len(Rover.nav_angles) >= Rover.go_forward and steerAngle != None and isNavigable(Rover, steerAngle):
        return True
    else:
        return False