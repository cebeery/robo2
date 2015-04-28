#!/usr/bin/env python

import pygame
from pygame.locals import *
import time
from pyx import path, unit, deformer
import copy
import math
import pickle

# for pyx (Trajectory.arclen)
# ALL UNITS m (distance), m/s (velocity), and s (time)
unit.set(defaultunit="m")
metersPerPoint = 0.00035277


""" METHODS"""

''' makeTrajectory:
    Show the user a view & make a trajectory from input
'''
def makeTrajectory(loadfile=None, savefile=None):
    if loadfile:
        with open('trajectories/'+loadfile+'.p', 'rb') as handle:
            traj = pickle.load(handle)
            traj.makePath() # remake path object...can't pickle

    else:
        pygame.init()

        """__________Initiates Class Objects and Umbrella Variables___________"""

        #Creates display surface
        size = (1000,600)
        screen = pygame.display.set_mode(size) 
        
        #Creates objects to see and modify virtual world
        vel = 100 # forward velocity of quadcopter, m/s
        traj = Trajectory(vel)
        view = View(traj,screen)

        #denote if pygame screen should be visible
        running = True
        
        
        """__________________________Draw/Drag Paths__________________________"""
      
        while running:
            for event in pygame.event.get():
                #End of World Conditions
                if event.type == QUIT:
                    running = False
                if event.type == KEYUP and event.key == pygame.K_ESCAPE:
                    running = False
                running = traj.update(event)

            view.draw()

            time.sleep(.005)
            
            
        """________________________Hold Path on Screen_________________________"""
      
        view.interpolate()
        running = True
        
        #Display Graph until End of World
        while running:
            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False
                if event.type == KEYUP and event.key == pygame.K_ESCAPE:
                    running = False

        pygame.quit()

        # Make PyX path and drop specified number of keyframes
        traj.makePath()
        traj.makeKeyframes(100)

        if savefile:
            with open('trajectories/'+savefile+'.p', 'wb') as handle:
                traj.path = None # can't pickle path objects
                pickle.dump(traj, handle)

    return traj



""" CLASSES """

''' Trajectory:
    Stores points from user input & translates into evenly spaced keyframes 
'''
class Trajectory:
    def __init__(self,velocity):
        self.velocity = velocity
        self.points = []
        self.draw = None
        self.path = None
        self.arclen = None
        self.duration = None # arclen/velocity
        self.keyframes = {'pos': [], 't': [], 'thdot': []} # pos = (x,y) position pairs (m); th = angles (deg), t = times (sec)


    def addPoints(self): 
        ''' stores a point from mouse position in pygame window '''
        x,y = pygame.mouse.get_pos()

        if not self.points:
            self.points.append((x,y))
        elif x != self.points[-1][0] or y != self.points[-1][1]:
            self.points.append((x,y)) # flip y axis


    def update(self, event):
        ''' watches pygame window for events '''
        if event.type == MOUSEBUTTONDOWN: 
            self.draw = True      
        elif event.type == MOUSEBUTTONUP:          
            self.draw = False
            
        if self.draw == True:
            self.addPoints()

        if self.draw == False:
            return False
        else:
            return True


    def makePath(self):
        ''' makes pyx path object from pygame points '''
        # so we can use pop() w/o modifying self.positions
        positions = copy.copy(self.points)
        positions.reverse()

        p = None

        # BEZIER CURVES
        while len(positions) > 4: # need at least 4 points to make bezier curve
            # make a bezier curve
            points = []
            for _ in range(3):
                points += positions.pop()
            points += positions[-1] # so next curve will start at this point

            if p:
                p += path.curve(*points)
            else:
                p = path.curve(*points)


        # LINES
        while len(positions) > 1: # use up rest of points with lines
            points = []
            points += positions.pop()
            points += positions[-1]

            if p:
                p += path.line(*points)
            else:
                p = path.line(*points)


        # store curve in object
        p = deformer.smoothed(2.0).deform(p) # smooth curve
        self.path = p
        self.arclen = p.arclen_pt()*metersPerPoint
        self.duration = self.arclen/self.velocity


    def makeKeyframes(self, n):
        ''' drops n keyframes along the trajectory 
            no return -> stores to self.keyframes
        '''
        self.keyframes = {'pos': [], 't': [], 'thdot': []} # clear, so method can be reused
        
        for i in range(n):
            timestep = self.duration/n # time between keyframes, seconds
            percentLength = i/float(n)

            # TIME from duration * percentLength
            self.keyframes['t'].append(self.duration*percentLength) # time
            
            # POSITION from arclen * percentLength
            point = self.path.at(self.arclen * percentLength)
            x = unit.tom(point[0]) # tom = convert to meters
            y = unit.tom(point[1])
            self.keyframes['pos'].append((x,y))

            # THETA from previous & next keyframes (working 1 frame behind pos)
            pos = self.keyframes['pos']

            if len(pos) <= 1: # working 1 index behind time/pos (need next position to compute angle)
                pass

            elif len(pos) <= 3: # don't have enough info to compute real angle yet
                self.keyframes['thdot'].append(0)

            else: # have enough info! Compute angle.
                # previous, current, & next positions
                prv = pos[-3]
                cur = pos[-2]
                nxt = pos[-1]

                # x and y deltas
                dx1 = cur[0]-prv[0]
                dx2 = nxt[0]-cur[0]

                dy1 = -(cur[1]-prv[1]) # flip b/c pygame coordinatization
                dy2 = -(nxt[1]-cur[1])

                # angles
                th1 = self.computeTheta(dx1, dy1)
                th2 = self.computeTheta(dx2, dy2)
            
                if self.getQuadrants([th1, th2]) == [1,4]: # adjust for q 1/4 weirdness
                    if th1 > th2: # going from q4 to q1, counterclockwise
                        change_th = th2 - (th1-360) # e.g. 359 to 1 = 2 -> 1-(359-360) = 2
                    else: # going from q1 to q4, clockwise
                        change_th = (th2-360) - th1 # e.g. 1 to 359 = -2 -> (359-360)-1 = -2
                else:
                    change_th = th2 - th1

                thdot = change_th/timestep
                self.keyframes['thdot'].append(thdot)

        # add last keyframe -> catch up to time & position
        self.keyframes['thdot'].append(0)


    def computeTheta(self, x, y):
        ''' returns angle ccw from x axis for (x,y) point
            handles quadrant math...atan has a range of (-pi/2, pi/2) 
        '''
        if x == 0:
            theta = math.atan(y/0.001)
        else:
            theta = math.atan(y/x)
            if x < 0:
                if y > 0:
                    theta = 3*math.pi/2 - (math.pi/2 + abs(theta))
                else:
                    theta = math.pi + abs(theta)

        while theta < 0:
            theta = theta + 2*math.pi

        return math.degrees(theta)


    def getQuadrants(self, angleList):
        quadrants = []
        for angle in angleList:
            quadrants.append(int(math.floor(angle/90) + 1))
        return sorted(quadrants)

''' View:
    Visualizes points from user input
'''
class View:
    """ A view rendered in a Pygame window """
    def __init__(self,model,screen):
        self.points = model.points
        self.screen = screen
        self.screen.fill(pygame.Color(255,255,255))
           

    def draw(self):
        """ """
        p = self.points 

        if p:
            pos = ( p[-1] )
            color = (0,0,0)
            self.screen.set_at(pos,color)
       
        pygame.display.update()


    def interpolate(self):
        """ """
        self.screen.fill(pygame.Color(255,255,255))
 
        #lines
        color = (50,50,50)
        closed = False
        pygame.draw.lines(self.screen, color, closed, self.points, 2)

        #dots
        dot = pygame.Surface((2,2))
        color = (0,220,220) 
        pygame.draw.circle(dot, color, (1,1), 1)
        dot.set_colorkey((0,0,0))   

        for pos in self.points:
            self.screen.blit(dot, (pos))
      
        pygame.display.update()



""" MAIN METHOD """            
if __name__ == '__main__':
    makeTrajectory()