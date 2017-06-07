# -*- coding: utf-8 -*-

import numpy as np
from perception import *

VISIT_DIST = 30

def mark_visited(Rover):
    for to in Rover.not_visited:
        dist = distance(Rover.pos, to)
        if dist < VISIT_DIST:
            print(to, 'visited')
            Rover.visited.add(to)
    Rover.not_visited = Rover.not_visited - Rover.visited


def is_on_border(mp, pos):
    for di in range(-1, 2):
        for dj in range(-1, 2):
            if mp[pos[0] + di, pos[1] + dj] == 0.0:
                return True
    return False

class NavRange:
    '''
    We want no obstacle in the front
    and left wall closer than right
    and left wall ideally in the range of 10 to 20
    '''
    def __init__(self, Rover):
        self.Rover = Rover
        self.angles_deg = self.Rover.nav_angles_deg()
        self.int_angles = self.angles_deg.astype(int)
        self.int_dists = self.Rover.nav_dists.astype(int)
        self.Rover.moves.append((int(self.Rover.pos[0]), int(self.Rover.pos[1])))
        while len(self.Rover.moves) > 5:
            del self.Rover.moves[0]
            
        self.moving = True
        if len(self.Rover.moves) == 5:
            allsame = True
            for i in range(1, len(self.Rover.moves)):
                if self.Rover.moves[0] != self.Rover.moves[i]:
                    allsame = True
            self.moving = not allsame
            if not self.moving:
                self.Rover.moves = []
        
        self.angles_dists = {}
        
        for i in range(len(self.int_angles)):
            angle = self.int_angles[i]
            dist = self.int_dists[i]
            if angle in self.angles_dists:
                if not dist in self.angles_dists[angle]:
                    self.angles_dists[angle].append(dist)
            else:
                self.angles_dists[angle] = [dist]
                
        #print (np.max(self.int_angles), np.min(self.int_angles))
        #print (np.max(self.int_dists), np.min(self.int_dists))
        
        self.free_to_go_range = []
        self.front_blocked = False
        
        range_start = None
        for angle in range(-50, 51):
            if angle in self.angles_dists:
                dists = self.angles_dists[angle]
                min_d = min(dists)
                max_d = max(dists)
                if min_d < 10 and max_d > min_d + 10:
                    #print (angle, 'is probably free to go')
                    if range_start == None:
                        range_start = angle
                else:
                    if angle == 0:
                        self.front_blocked = True
                    #print (angle, 'is probably blocked')
                    if range_start != None:
                        if (angle - range_start) > 3:
                            navigable_range = (range_start, angle)                        
                            print ('navigable range', navigable_range)
                            self.free_to_go_range.append(navigable_range)
                        range_start = None
                #print (angle, min_d, max_d)
                
        print(self.free_to_go_range)
        
#        self.closest_pt = closest_point_on_path(self.Rover)
#        #print('closest', closest_pt)
#        
#        self.next_pts = get_next_points(self.Rover, self.closest_pt)
#        print('next points', self.next_pts)
#        
#        if self.closest_pt != None and self.closest_pt in self.Rover.obspos:
#            print('target beacon is in obstacle zone, mark visited.')
#            self.Rover.visited.add(self.closest_pt)
#            self.Rover.not_visited = self.Rover.not_visited - self.Rover.visited
            
        self.recommended_angle = None
        if len(self.Rover.gold_angles) != 0:
            self.recommended_angle = self.Rover.gold_angles_deg()[0]
            print ('gold angle', self.recommended_angle)
#        if self.next_pts != None and len(self.next_pts) > 0:
#            adeg = np.zeros(len(self.next_pts))
#            for i in range(len(self.next_pts)):
#                adeg[i] = self.next_pts[i][1]
#            print('avg over', adeg)
#            self.recommended_angle = int(np.mean(adeg))
#            
#        print('recommended angle from path', self.recommended_angle)
        
    def getNavAngleClosestToRecommended(self):
        nav_angle = None
        if self.recommended_angle != None:
            mindiff = None
            for r in self.free_to_go_range:
                angle = r[0] + (r[1] - r[0])/2
                if mindiff == None or abs(angle - self.recommended_angle) < mindiff:
                    mindiff = abs(angle - self.recommended_angle)
                    nav_angle = angle
        else:
            max_angle = 0
            nav_range = None
            for r in self.free_to_go_range:
                width = r[1] - r[0]
                if width > max_angle:
                    max_angle = width
                    nav_range = r
                    nav_angle = nav_range[0] + (nav_range[1] - nav_range[0])/2
                    
        return nav_angle
        
    def getNavAngle(self):
        self.has_nav_angle = False
        nav_angle = self.getNavAngleClosestToRecommended()
        if not self.moving:
            print ('not moving, get out of here')
            self.has_nav_angle = True
            return -2
        elif nav_angle != None and not self.front_blocked:
            print ('navigate to angle', nav_angle)
            self.has_nav_angle = True
            return nav_angle
        else:
            print ('get out of here')
            return -15


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        
        navRange = NavRange(Rover)
        steer_angle = navRange.getNavAngle()
#        for i in range(len(Rover.nav_angles)):
#            if is_on_border(Rover.mp, Rover.navpos[i]):
#                print (Rover.nav_angles[i], Rover.nav_dists[i])
#        mark_visited(Rover)
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward and navRange.has_nav_angle:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(steer_angle, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward or not navRange.has_nav_angle:
                if navRange.moving:
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
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward or not navRange.has_nav_angle:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward and navRange.has_nav_angle:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(steer_angle, -15, 15)
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
