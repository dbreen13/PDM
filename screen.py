# -*- coding: utf-8 -*-
"""
Created on Tue May 16 14:34:15 2023

@author: demib
"""
import time
import math
import matplotlib.pyplot as plt
import numpy as np
from sklearn.preprocessing import normalize
# import pygame module in this program
import pygame
import pygame.freetype  # Replacement of pygame.font with more functionality
# import serial
# import scipy
# from serial.tools import list_ports
# from scipy.spatial.distance import cdist
# from pantograph import Pantograph
# from pshape import PShape
# from pyhapi import Board, Device, Mechanisms
# from PIL import Image

pygame.init()

interface = pygame.display.set_mode((1600, 900))  # Twice 600x400 for haptic and VR (horizontal layout)
pygame.display.set_caption('Visual guidance')  # Set pygame window title

#create a pygame surface
background_image = pygame.image.load("page.png").convert()
screen=pygame.Surface((1600, 900))
white = (255, 255, 255)
green = (0, 255, 0)
red=(255, 0, 0)
black=(0,0,0)

#get the borders of the screen and center
w, h = interface.get_size()
width, height = screen.get_size()
xc, yc = screen.get_rect().center  ##center of the screen
trajectory=[]
#show mouse
pygame.mouse.set_visible(True)  # Hide cursor by default. 'm' toggles it

# Initialize "real-time" clock
clock = pygame.time.Clock()
FPS = 100  # in Hertz
# Get the current time in milliseconds
current_time = pygame.time.get_ticks()

#create a rectanlge
mouse = pygame.Rect(*screen.get_rect().center, 0, 0).inflate(2, 2) #change to 48 if you want pen visualization
cursor = pygame.Rect(0, 0, 10, 10)
xh = np.array(mouse.center)
##Set the old value to 0 to avoid jumps at init
m_old = 0
t_prev = 0

point=np.array([0,0])

#initialize motor array
motor=np.array([0,0,0,0])
# Wait until the start button is pressed
run = True
while run:
    for event in pygame.event.get():  # Interrupt function
        if event.type == pygame.KEYUP:

            if event.key == ord('e'):  # Enter the main loop after 'e' is pressed
                run = False
                


pygame.display.flip()
# Draws the surface object to the screen.
pygame.display.update()

run = True
t_prev = time.time()


while run:
    st=time.time()
    for event in pygame.event.get():
        ##If the window is close then quit
        if event.type == pygame.QUIT:
            run = False
        elif event.type == pygame.KEYUP:
            if event.key == ord('m'):  ##Change the visibility of the mouse
                pygame.mouse.set_visible(not pygame.mouse.get_visible())
            if event.key == ord('q'):  ##Force to quit
                run = False
            if event.key == ord('p'):
                coord_x=np.random.choice(width, size=1)
                coord_y=np.random.choice(height, size=1)
                point=np.array([coord_x[0],coord_y[0]])
            
                
    ##Compute distances and forces between blocks
    xh = np.clip(np.array(mouse.center), 0, 1600)
    xh = np.round(xh)

    ##Get mouse position    
    mouse_pos=pygame.mouse.get_pos()
    trajectory.append(mouse_pos)
    
    cursor.center=mouse_pos
    xm = np.clip(np.array(cursor.center), 0, 1600)
    
    dis=mouse_pos-point
    
    vector_norm=dis/np.linalg.norm(dis)
   
    if vector_norm[0]<0 and vector_norm[1]>0: #in between motor 1 and 2
        motor=np.array([0,0,0,0])
        motor[0]=(np.abs(vector_norm[1])*255)/25
        motor[1]=(np.abs(vector_norm[0])*255)/25
        motor.astype(int)
    elif vector_norm[0]<0 and vector_norm[1]<0:  #in between motor 2 and 3
        motor=np.array([0,0,0,0])
        motor[1:3]=(np.abs(vector_norm)*255)/25
        motor.astype(int)
    elif vector_norm[0]>0 and vector_norm[1]<0:  #in between motor 3 and 4
        motor=np.array([0,0,0,0])
        motor[2]=(np.abs(vector_norm[1])*255)/25
        motor[3]=(np.abs(vector_norm[0])*255)/25
        motor.astype(int)
    elif vector_norm[0]>0 and vector_norm[1]>0:  #in between motor 1 and 4
        motor=np.array([0,0,0,0])
        motor[-1]=(np.abs(vector_norm[0])*255)/25
        motor[0]=(np.abs(vector_norm[1])*255)/25
        motor.astype(int)
    elif vector_norm[0] == 0.0 and vector_norm[1] >0:
        motor=np.array([0,0,0,0])
        motor[0]=(np.abs(vector_norm[1])*255)/25
        motor.astype(int)
    elif vector_norm[0]  == 0.0 and vector_norm[1] >0:
        motor=np.array([0,0,0,0])
        motor[2]=(np.abs(vector_norm[1])*255)/25
        motor.astype(int)
    elif vector_norm[0] < 0 and vector_norm[1] ==0.0:
        motor=np.array([0,0,0,0])
        motor[1]=(np.abs(vector_norm[0])*255)/25
        motor.astype(int)
    elif vector_norm[0] > 0 and vector_norm[1]  ==0.0:
        motor=np.array([0,0,0,0])
        motor[3]=(np.abs(vector_norm[0])*255)/25
        motor.astype(int)
    elif vector_norm[0]  ==0 and vector_norm[1]  ==0:
        motor=np.array([0,0,0,0])
    print(motor)
   
    ## visualization    
    screen.fill(white)
    #screen.blit(background_image, (0, 0))
    pygame.draw.rect(screen,green, mouse, border_radius=1)
    pygame.draw.rect(screen, green, cursor) 
    pygame.draw.line(screen,black,xm,np.array([xm[0]-dis[0]/10,xm[1]-dis[1]/10]))
    point_rect=pygame.Rect(point[0]-15, point[1]-15, 30, 30)
    pygame.draw.rect(screen, red, point_rect)
    
    
    interface.blit(screen, (0, 0))
    
    pygame.display.flip()
    # Draws the surface object to the screen.
    pygame.display.update()
    ##Slow down the loop to match FPS
    clock.tick(FPS)
    et=time.time()

pygame.display.quit()
pygame.quit()


