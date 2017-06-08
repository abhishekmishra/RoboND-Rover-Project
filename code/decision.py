import numpy as np
from random import randint

#def subset_angles_dists(angles, dists, angle_pred):
#    s_angles = []
#    s_dists = []
#    for i in range(len(angles)):
#        if angle_pred(angles[i]):
#            s_angles.append(angles[i])
#            s_dists.append(dists[i])
#            
#    return s_angles, s_dists
#
#def already_seen(Rover, pos):
#    x, y = pos[0], pos[1]
#    return Rover.worldmap[x, y, 2] > 0 or Rover.worldmap[x, y, 1] > 0 or Rover.worldmap[x, y, 0] > 0
#
#def get_unseen_angles(Rover):
#    int_angles = Rover.nav_angles_deg().astype(int)
#    int_dists = Rover.nav_dists.astype(int)
#    angles = []
#    dists = []
#    for i in range(len(Rover.navpos)):
#        if not already_seen(Rover, Rover.navpos[i]):
#            angles.append(int_angles[i])
#            dists.append(int_dists[i])
#    return angles, dists

def dist_at_range(angles, dists, r=range(35, 40)):
    total = 0
    for x in r:
        max_dist = 0
        for i in range(len(angles)):
            if angles[i] == x and dists[i] > max_dist:
                max_dist = dists[i]
        #print('at', x, 'max=', max_dist)
        total += max_dist
    return int(total/len(r))
                
# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    
    
    #left_angles, left_dists = subset_angles_dists(int_angles, int_dists, lambda x: x >= 0)
    #right_angles, right_dists = subset_angles_dists(int_angles, int_dists, lambda x: x < 0)
    
    #unseen_angles, unseen_dists = get_unseen_angles(Rover)
    #print (unseen_angles)
    
    if Rover.nav_angles is not None and len(Rover.nav_angles) > 0:
        int_angles = Rover.nav_angles_deg().astype(int)
        int_dists = Rover.nav_dists.astype(int)

        seen_gold = False
        #print(Rover.gold_angles)
        if Rover.gold_angles is not None and len(Rover.gold_angles) > 0:
            seen_gold = True
        
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if Rover.stuck():
                print('forward: rover stuck')
                Rover.throttle = 0
                Rover.steer = -15
                Rover.mode = 'stuck'
#            elif len(unseen_angles) >= Rover.stop_forward:
#                print ('we have unseen pixels')
#                if Rover.vel < Rover.max_vel:
#                    Rover.throttle = Rover.throttle_set
#                else: # Else coast
#                    Rover.throttle = 0
#                Rover.brake = 0
#                # Set steering to average angle clipped to the range +/- 15
#                Rover.steer = np.clip(np.mean(unseen_angles), -15, 15)
            elif seen_gold and not Rover.near_sample \
                and np.min(Rover.gold_angles) > np.min(Rover.nav_angles) \
                and np.max(Rover.gold_angles) < np.max(Rover.nav_angles):
                print('forward: seen gold, and not near sample')
                dist_to_gold = Rover.gold_dists[0]
#                print('steer', np.mean(Rover.gold_angles * 180/np.pi))
#                print('dist to gold', dist_to_gold)
                if Rover.vel > 0.5:
                    print('forward: too fast for gold')
                    Rover.throttle = -1
                    Rover.brake = 10
                elif Rover.vel < 0.3:
                    print('forward: too slow for gold')
                    Rover.throttle = 0.1
                    Rover.brake = 0
                else:
                    print('forward: correct speed') 
                    Rover.throttle = 0
                    Rover.brake = 0
#                print(Rover.gold_angles * 180/np.pi)
#                print(np.mean(Rover.gold_angles * 180/np.pi))
                Rover.steer = np.clip(np.mean(Rover.gold_angles * 180/np.pi), -15, 15)
            elif Rover.near_sample:
                print('forward: near sample, start stopping')
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                dist_from_wall = dist_at_range(int_angles, int_dists)
                dist_from_front_wall = dist_at_range(int_angles, int_dists, range(-5, 5))
                print('forward: enough angles move forward, dist_from_wall', dist_from_wall)
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                safe_dist = 10.0
                if Rover.vel < Rover.max_vel and dist_from_front_wall > safe_dist:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                elif dist_from_front_wall < safe_dist:
                    Rover.throttle = 0
                else: # Else coast
                    Rover.throttle = 0
                sdratio = safe_dist/15
                Rover.brake = 0
                if dist_from_wall < safe_dist:
#                    steer_angle = np.mean(Rover.nav_angles * 180/np.pi) - 5
#                elif dist_from_wall < 30:
                    steer_angle = np.mean(Rover.nav_angles * 180/np.pi)
                elif dist_from_wall > safe_dist and dist_from_wall < (2 * safe_dist):
                    steer_angle = np.mean(Rover.nav_angles * 180/np.pi) - dist_from_wall/sdratio
                else:
                    steer_angle = np.max(Rover.nav_angles * 180/np.pi) - randint(0, 5)
                #max_angle = np.max(Rover.nav_angles * 180/np.pi)
                #diff_from_left = 50 - max_angle
                #steer_angle = max_angle - diff_from_left/2
                #print('max, dist_from_left, steer', max_angle, diff_from_left, steer_angle)
                if steer_angle < np.min(Rover.nav_angles):
                    steer_angle = np.mean(Rover.nav_angles * 180/np.pi)
                #    print('steer angle not in range, taking mean', steer_angle)
                Rover.steer = np.clip(steer_angle, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                print('forward: no angles stop')
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop' or Rover.mode == 'stuck':
            # If we're in stop mode but still moving keep braking
            if Rover.stuck():
                Rover.throttle = 0
                Rover.steer = -15
                Rover.mode = 'stuck'
            elif Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
#                if len(unseen_angles) >= Rover.stop_forward:
#                    if Rover.vel < Rover.max_vel:
#                        Rover.throttle = Rover.throttle_set
#                    else: # Else coast
#                        Rover.throttle = 0
#                    Rover.brake = 0
#                    # Set steering to average angle clipped to the range +/- 15
#                    Rover.steer = np.clip(np.mean(unseen_angles), -15, 15)
                if seen_gold and not Rover.near_sample:
                    print('stop: seen gold, and not near sample')
                    dist_to_gold = Rover.gold_dists[0]
#                    print('steer', np.mean(Rover.gold_angles * 180/np.pi))
#                    print('dist to gold', dist_to_gold)
#                    print('too slow for gold')
                    Rover.throttle = 0.1
                    Rover.brake = 0
#                    print(Rover.gold_angles * 180/np.pi)
#                    print(np.mean(Rover.gold_angles * 180/np.pi))
                    Rover.steer = np.clip(np.mean(Rover.gold_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                elif Rover.near_sample:
                    print('stop: near sample, start stopping')
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                elif len(Rover.nav_angles) < Rover.go_forward:
                    print('stop: stop and turn left')
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(Rover.nav_angles) >= Rover.go_forward:
                    print('stop: move forward')
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        if Rover.stuck():
            Rover.throttle = 0
            Rover.steer = -15
            Rover.mode = 'stuck'
        else:
            Rover.throttle = Rover.throttle_set
            Rover.steer = 0
            Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover
