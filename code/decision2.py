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
                Rover.steer = -15#getSteerAngle(Rover)
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

    
SMALL_DIST = 30

class NavRange:
    '''
    A class to calculate navigable angles
    and distances from the given angles and dists list
    
    For each navigable angle we store a sorted list of
    distances navigable in that angle
    '''
    def __init__(self, angles, dists):
        self.angles_dists = {}
        for i in range(len(angles)):
            angle = int(angles[i])
            dist = int(dists[i])
            if not angle in self.angles_dists.keys():
                self.angles_dists[angle] = []
            d = self.angles_dists[angle]
            loc = 0
            while loc < len(d) and dist > d[loc]:
                loc += 1
            if loc >= len(d):
                d.append(dist)
            else:
                d.insert(loc, dist)
        #print(self.angles_dists)
            

    def left_and_right_angles(self):
        left_angles = []
        left_dists = []
        right_angles = []
        right_dists = []
        for angle, dists in self.angles_dists.items():
            for dist in dists:
                if angle >= 0:
                    left_angles.append(angle)
                    left_dists.append(dist)
                else:
                    right_angles.append(angle)
                    right_dists.append(dist)
    
        return (NavRange(left_angles, left_dists), NavRange(right_angles, right_dists))
    
    def filter_small_distances(self):
        self.filtered_angles_dists = {}

        angles_to_remove = []
        for angle, dists in self.angles_dists.items():
            if min(dists) > SMALL_DIST or max(dists) < SMALL_DIST:
                angles_to_remove.append(angle)
        for angle, dists in self.angles_dists.items():
            if not angle in angles_to_remove:
                self.filtered_angles_dists[angle] = dists
    
    def obstacle_in_front(self):
        angles = np.array(list(self.filtered_angles_dists.keys()))
        max_angle = np.max(angles)
        min_angle = np.min(angles)
        #print('max', max_angle, 'min', min_angle)
        
#        if (max_angle <= 0 and min_angle <= 0):
#            print('both angles are negative')
#            return True
#        elif (max_angle > 0 and min_angle > 0):
#            print('both angles are positive')
#            return True
        not_found_count = 0
        for i in range(min_angle, max_angle):
            if not i in angles:
                not_found_count += 1
            else:
                not_found_count = 0
            if not_found_count > 5:
                #print(self.filtered_angles_dists)
                print(i, 'is blocked')
                return True
        return False

    def angles_arr(self):
        angles = list(self.angles_dists.keys())
        arr = np.array(angles)
        return arr
    
    def wall_hugging_angle(self):
        angles = self.angles_arr()
        whangle = np.mean(angles)
        return whangle
    
    def max_dists_mean(self):
        max_dists = []
        for k, v in self.angles_dists.items():
            max_dists.append(max(v))
            
        return np.mean(np.array(max_dists))
   

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
    dists = Rover.nav_dists
    navRange = NavRange(angles, dists)
    navRange.filter_small_distances()
    
    steer_angle = None
    
    if len(navRange.filtered_angles_dists.keys()) == 0 :
        print('No navigable angles')
    elif navRange.obstacle_in_front():
        print('obstacle in front')
    else:
        steer_angle = navRange.wall_hugging_angle()
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