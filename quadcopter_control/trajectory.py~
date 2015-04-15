#!/usr/bin/env python

import pygame
from pygame.locals import *
import time


""" """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """
Classes
""" """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """

class MakeTrajectory:
    def __init__(self,screen):
        self.screen = screen
        self.points = []
        self.draw = None
       
    def AddPoints(self): 

        x,y = pygame.mouse.get_pos()

        if not self.points:
            self.points.append(x)
            self.points.append(y)
        elif x != self.points[-2] or y != self.points[-1]:
            self.points.append(x)
            self.points.append(y)

    def update(self, event):
        
        if event.type == MOUSEBUTTONDOWN: 
            self.draw = True      
        elif event.type == MOUSEBUTTONUP:          
            self.draw = False
            
        if self.draw == True:
            self.AddPoints()

        if self.draw == False:
            return False
        else:
            return True


"""  """  """  """  """  """  """  """  """  """  """  """  """  """  """
View
"""  """  """  """  """  """  """  """  """  """  """  """  """  """  """
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
            pos = ( p[-2], p[-1] )
            color = (0,0,0)
            self.screen.set_at(pos,color)
       
        pygame.display.update()

    def interpolate(self):
        """ """
        self.screen.fill(pygame.Color(255,255,255))
        p = self.points 

        pointlist = []
        for i in range(0, len(p), 2):
            pointlist.append( (p[i], p[i+1]) )
            
        #lines
        color = (50,50,50)
        closed = False
        pygame.draw.lines(self.screen, color, closed, pointlist,2)

        #dots
        dot = pygame.Surface((2,2))
        color = (0,220,220) 
        pygame.draw.circle(dot, color, (1,1), 1)
        dot.set_colorkey((0,0,0))   

        for pos in pointlist:
            self.screen.blit(dot, (pos))

      
        pygame.display.update()


""" """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """
Running Code
""" """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """ """
            
if __name__ == '__main__':

    pygame.init()

    """__________Initiates Class Objects and Umbrella Variables___________"""

    #Creates display surface
    size = (1000,600)
    screen = pygame.display.set_mode(size) 
    
    #Creates objects to see and modify virtual world
    mTraj = MakeTrajectory(screen)
    view = View(mTraj,screen)

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
            running = mTraj.update(event)

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
