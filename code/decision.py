import numpy as np

from perception import *

#WALL_TOO_CLOSE = 3
VISIT_DIST = 20

def mark_visited(Rover):
    for to in Rover.not_visited:
        dist = distance(Rover.pos, to)
        if dist < VISIT_DIST:
            Rover.visited.add(to)
    Rover.not_visited = Rover.not_visited - Rover.visited

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    
    #setup rover path if not already set
    
    closest_pt = closest_point_on_path(Rover)
    #print('closest', closest_pt)
    
    next_pts = get_next_points(Rover, closest_pt)
    print('next points', next_pts)
    
    if closest_pt != None and closest_pt in Rover.obspos:
        print('target beacon is in obstacle zone, mark visited.')
        Rover.visited.add(closest_pt)
        Rover.not_visited = Rover.not_visited - Rover.visited
    
    stop_and_turn = False
    if Rover.nav_angles is None or len(Rover.nav_angles) < Rover.stop_forward or np.mean(Rover.int_dists) < 5:
        print('obstacle in front... stop_and_turn')
        #print (Rover.obspos)
        stop_and_turn = True
        steer_angle = 15
    elif next_pts != None and len(next_pts) > 0:
        adeg = np.zeros(len(next_pts))
        for i in range(len(next_pts)):
            adeg[i] = next_pts[i][1]
        print('avg over', adeg)
        target_angle = np.mean(adeg)
        if target_angle > 45 or target_angle < -45:
            print ('target angle not in [-45, 45]')
            stop_and_turn = True
        elif target_angle > np.max(Rover.nav_angles_deg()) or target_angle < np.min(Rover.nav_angles_deg()):
            print ('target angle not in nav range')
            #print (Rover.obspos)
            stop_and_turn = True
        steer_angle = np.clip(target_angle, -15, 15)
        print ('target angle', target_angle)
    else:
        steer_angle = np.clip(np.mean(Rover.nav_angles_deg()), -15, 15)
    print ('steer angle', steer_angle)
    
    mark_visited(Rover)
    
    if Rover.gold_angles is not None:
        if len(Rover.gold_angles) > 0:
            print('gold ahead')
    
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward and not stop_and_turn:
                print('moving: enough angles and no stop_and_turn')
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = steer_angle
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            else:#if len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                print('moving: need to stop')
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
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    print('stopped: not enough angles')

                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    #if len(navRange.nav_angles_deg) == 0 or np.min(navRange.nav_angles_deg) < 0:
                    #    Rover.steer = -15
                    #else:
                    Rover.steer = steer_angle
#                    if np.sign(steer_angle) > 0:
#                        Rover.steer = 15
#                    else:
#                        Rover.steer = -15
                    # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif stop_and_turn:
                    print('stopped: stop_and_turn', steer_angle)
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = steer_angle
                else:
                    print('stopped: move forward', steer_angle)
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = steer_angle
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover
