import numpy as np

#WALL_TOO_CLOSE = 3
MIN_DIST_TO_MOVE = 10

MOVE_HISTORY = 20

#min x which is unblocked
def get_min_x_unblocked(mp):
    for i in range(199, -1, -1):
        for j in range(199, -1, -1):
            if mp[i, j] > 0.0:
                return i, j
#lowest_ub = get_min_x_unblocked(mp)
#print(lowest_ub)

def is_on_border(mp, pos):
    for di in range(-1, 2):
        for dj in range(-1, 2):
            if mp[pos[0] + di, pos[1] + dj] == 0.0:
                return True
    return False
#is_on_border(mp, lowest_ub)

DIST_FROM_WALL = 2

def median_path(mp, left_right = True):
    lmeans = []
    rmeans = []
    for i in range(mp.shape[0]):
        start = None
        end = None
        inside = False
        for j in range(mp.shape[1]):
            if left_right == True:
                pos = (i, j)
            else:
                pos = (j, i)
            if mp[pos] == 1.0 and not inside:
                inside = True
                start = j
            if mp[pos] == 0.0 and inside:
                inside = False
                end = j
            if start != None and end != None:
                wh = start + DIST_FROM_WALL
                if left_right == True:
                    new_pos = (i, wh)
                else:
                    new_pos = (wh, i)
                if mp[new_pos] == 1.0:
                    lmeans.append(new_pos)
                
                wh = end - DIST_FROM_WALL
                if left_right == True:
                    new_pos = (i, wh)
                else:
                    new_pos = (wh, i)
                if mp[new_pos] == 1.0:
                    rmeans.append(new_pos)
                
                start = None
                end = None
    return lmeans + rmeans

def distance(pos1, pos2):
    return np.sqrt(np.square(pos1[0] - pos2[0]) + np.square(pos1[1] - pos2[1]))

def sign_changes(start, end, new_pos):
    sc = 0
    if np.sign(new_pos[0] - end[0]) != np.sign(end[0] - start[0]):
        sc += 1
    if np.sign(new_pos[1] - end[1]) != np.sign(end[1] - start[1]):
        sc += 1
    return sc

def find_min_not_visited(prev, pos, not_visited):
    min_dist = None
    min_pos = None
    min_sc = None
    for sc in range(3):
        for to in not_visited:
            dc = sign_changes(prev, pos, to)
            dist = distance(pos, to)
            if min_dist == None or (dc == sc and dist < min_dist and dist > 1):
                min_dist = dist
                min_pos = to
                min_sc = dc
        if min_dist != None and min_dist < 5:
            break
    if min_dist != None and min_dist < 30:
        #print(min_dist, prev, pos, min_pos, min_sc)
        return min_pos
    else:
        return None

def greedy_path(current_path):
    #visited = set([])
    new_path = []
    not_visited = set(current_path)
    current = current_path[0]
    prev = current
    while current != None and current in not_visited:
        not_visited.discard(current)
        new_path.append(current)
        prev = current
        current = find_min_not_visited(prev, current, not_visited)
    print('not visited', len(not_visited))
    return new_path

#lrmedians = median_path(mp)
#upmedians = median_path(mp, False)

#medians = lrmedians + upmedians

#apath = greedy_path(medians)
#print('path length', len(apath))



def updateMoves(Rover):
    if len(Rover.moves) == MOVE_HISTORY:
        del Rover.moves[0]
    Rover.moves.append((Rover.pos[0], Rover.pos[1]))
    #print('last 5 moves', Rover.moves)
    
def roverNotMoving(Rover):
    if Rover.throttle > 0:
        if len(Rover.moves) == MOVE_HISTORY:
            for i in range(1, MOVE_HISTORY):
                if Rover.moves[i][0] != Rover.moves[0][0]:
                    return False
                if Rover.moves[i][1] != Rover.moves[0][1]:
                    return False
            return True
        return False
    return False

class NavRange:
    
    def __init__(self, Rover):
        updateMoves(Rover)
        self.Rover = Rover
        self.nav_angles_deg = Rover.nav_angles_deg().astype(int)
        self.steer_angle = Rover.steer
        if len(self.nav_angles_deg) > 0:
            print ('current steer angle', Rover.steer)
            self.max_angle = np.max(self.nav_angles_deg)
            self.min_angle = np.min(self.nav_angles_deg)
            self.max_angle_dist = 0
            self.min_dist_from_left = None
            
            self.calculate_min_dists(Rover)
            self.calculate_max_dists(Rover)
            self.calculate_leftwall_and_rightwall()
            
#            for i in range(60, -61, -1):
#                if i in self.max_dist_by_angle:
#                    print('max at', i, self.max_dist_by_angle[i])
#                if i in self.min_dist_by_angle:
#                    print('min at', i, self.min_dist_by_angle[i])
#                else:
#                    print(i, 0)
            
            for i in range(len(self.nav_angles_deg)):
                if self.nav_angles_deg[i] == self.max_angle and Rover.nav_dists[i] > self.max_angle_dist:
                    self.max_angle_dist = Rover.nav_dists[i]
                if self.nav_angles_deg[i] > 20:
                    if self.min_dist_from_left == None or self.max_dist_by_angle[self.nav_angles_deg[i]] < self.min_dist_from_left:
                        min_dist_from_left = self.max_dist_by_angle[self.nav_angles_deg[i]]
                        
            print('navigable dist at max angle is', self.max_angle, self.max_angle_dist,
                  'min dist from left', self.min_dist_from_left)
            if self.max_angle < 0:
                print('im pointing towards the wall')
                self.steer_angle = np.clip(self.min_angle, -15, 15)
            elif self.left_dist_mean < 7: 
                #self.max_angle_dist < 10 or (min_dist_from_left != None and min_dist_from_left < 10):
                print('too close, turn slightly right')
                self.steer_angle = np.clip(np.mean(self.nav_angles_deg) - self.max_angle/5, -15, 15)
            elif self.left_dist_mean > 10:
                print('too far, turn slightly left')
                self.steer_angle = np.clip(np.mean(self.nav_angles_deg) + self.max_angle/5, -15, 15)
#            elif np.max(nav_angles_deg) > 20:
#                print('pointing towards the wall, take a mean angle')
#                steer_angle = np.clip(np.mean(nav_angles_deg) + max_angle/5, -15, 15)
            else:
                print('hugging wall, stay in the same direction')
                self.steer_angle = 0#Rover.steer
            self.steer_angle = int(self.steer_angle)

    def calculate_min_dists(self, Rover):
        self.min_dist_by_angle = {}
        for i in range(len(self.nav_angles_deg)):
            if self.nav_angles_deg[i] not in self.min_dist_by_angle:
                self.min_dist_by_angle[self.nav_angles_deg[i]] = Rover.nav_dists[i]
            elif Rover.nav_dists[i] < self.min_dist_by_angle[self.nav_angles_deg[i]]:
                self.min_dist_by_angle[self.nav_angles_deg[i]] = Rover.nav_dists[i]

    def calculate_max_dists(self, Rover):
        self.max_dist_by_angle = {}
        for i in range(len(self.nav_angles_deg)):
            if self.nav_angles_deg[i] not in self.max_dist_by_angle:
                self.max_dist_by_angle[self.nav_angles_deg[i]] = Rover.nav_dists[i]
            elif Rover.nav_dists[i] > self.max_dist_by_angle[self.nav_angles_deg[i]]:
                self.max_dist_by_angle[self.nav_angles_deg[i]] = Rover.nav_dists[i]

    def calculate_leftwall_and_rightwall(self):
        self.left_angles, self.left_dist_sum, self.left_dist_mean = self.angles_dist_in_range(0, 60)
        self.right_angles, self.right_dist_sum, self.right_dist_mean = self.angles_dist_in_range(-60, 0)

    def leftwall_too_close(self, dist = MIN_DIST_TO_MOVE):
        return len(self.left_angles) < 20 or self.left_dist_mean < dist
    
    def rightwall_too_close(self, dist = MIN_DIST_TO_MOVE):
        return len(self.right_angles) < 20 or self.right_dist_mean < dist
    
    def blocked_at_front(self):
        return (not 0 in self.nav_angles_deg) or self.min_dist_by_angle[0] > 6
    
    def angles_dist_in_range(self, start, end):
        angles = []
        dist_sum = 0
        dist_mean = 0
        for i in range(end, start - 1, -1):
            if i in self.max_dist_by_angle:
                angles.append(i)
                dist_sum += self.max_dist_by_angle[i]
        if len(angles) > 0:
            dist_mean = dist_sum/len(angles)
        return (angles, dist_sum, dist_mean)
    
    def angle_unblocked(self):
        if not self.steer_angle in self.max_dist_by_angle or self.max_dist_by_angle[self.steer_angle] < MIN_DIST_TO_MOVE:
            print ('steer angle is blocked')
            return False
        
        if self.steer_angle > np.max(self.nav_angles_deg) or self.steer_angle < np.min(self.nav_angles_deg):
            print ('steer angle is out of range', np.max(self.nav_angles_deg), np.min(self.nav_angles_deg))
            return False
        
        if self.leftwall_too_close() and self.rightwall_too_close():
            print ('both walls too close')
            return False
        
        if self.blocked_at_front():
            print ('blocked at front')
            return False
        
        if roverNotMoving(self.Rover):
            print ('rover is not moving')
            return False
        
        print ('angle', self.steer_angle, 'unblocked')
        return True

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    navRange = NavRange(Rover)
    if Rover.gold_angles is not None:
        if len(Rover.gold_angles) > 0:
            print('gold ahead')
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward and navRange.angle_unblocked():  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = navRange.steer_angle
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            else:#if len(Rover.nav_angles) < Rover.stop_forward:
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
                if len(Rover.nav_angles) < Rover.go_forward or not navRange.angle_unblocked():
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    #if len(navRange.nav_angles_deg) == 0 or np.min(navRange.nav_angles_deg) < 0:
                    #    Rover.steer = -15
                    #else:
                    Rover.steer = 15
                    # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                else:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = navRange.steer_angle
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

