#!/usr/bin/env python

import pygame
from pygame.locals import *
import time
from pyx import path, unit
import copy

# for pyx (Trajectory.arclen)
unit.set(defaultunit="m")
metersPerPoint = 0.00035277


""" METHODS"""

''' makeTrajectory:
    Show the user a view & make a trajectory from input
'''
def makeTrajectory():
    pygame.init()

    """__________Initiates Class Objects and Umbrella Variables___________"""

    #Creates display surface
    size = (1000,600)
    screen = pygame.display.set_mode(size) 
    
    #Creates objects to see and modify virtual world
    traj = Trajectory(screen)
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

        """
        The window to paradise is fluid, allowing us to bear it witness
        """        
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

    traj.makePath() # make pyx path tracing all points
    return traj



""" CLASSES """

''' Trajectory:
    Stores points from user input & translates into evenly spaced keyframes 
'''
class Trajectory:
    def __init__(self,screen):
        self.points = []
        self.draw = None
        self.path = None
        self.arclen = None


    def addPoints(self): 
        x,y = pygame.mouse.get_pos()

        if not self.points:
            self.points.append((x,y))
        elif x != self.points[-1][0] or y != self.points[-1][1]:
            self.points.append((x,y))


    def keyframe(self, percentLength):
    	''' percentLength = 0 -> keyframe at beginning of path
    		percentLength = 0.5 -> keyframe halfway along arc length
    		etc
    	'''
    	point = self.path.at(self.arclen * percentLength)

    	x = unit.tom(point[0]) # tom = convert to meters
    	y = unit.tom(point[1])

    	return (x,y)


    def update(self, event):
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
        self.path = p
        self.arclen = p.arclen_pt()*metersPerPoint



''' View:
    Visualizes points from user input
'''
class View:
    """ A view of Turtle's World rendered in a Pygame window """
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