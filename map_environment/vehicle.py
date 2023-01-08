import pygame as py  
import math

# define constants  
WIDTH = 550  
HEIGHT = 550  
FPS = 10  

# define colors  
BLACK = (0 , 0 , 0)  
GREEN = (0 , 255 , 0)  

# initialize pygame and create screen  
py.init()  
screen = py.display.set_mode((WIDTH , HEIGHT))  
# for setting FPS  
clock = py.time.Clock()  

#rot = 0  
#rot_speed = 2

car_color = (255, 0, 0)
car_size = (20, 30)

#Test MPC-values
x_mpc = (0 , 20, 40, 60, 80, 100, 120, 140, 160, 180, 200)
y_mpc = (0, 10, 30, 60, 100, 150, 210, 280, 360, 450, 550)
phi_mpc = [0,  0.52359878,  1.04719755,  1.57079633,  2.0943951 , 2.61799388,  3.14159265,  3.66519143,  4.1887902 ,  4.71238898, 5.23598776]

# define a surface (RECTANGLE)  
image_orig = py.Surface(car_size)  
# for making transparent background while rotating an image  
image_orig.set_colorkey(BLACK)  
# fill the rectangle / surface with green color  
image_orig.fill(car_color)  
# creating a copy of orignal image for smooth rotation  
image = image_orig.copy()  
image.set_colorkey(BLACK)  
# define rect for placing the rectangle at the desired position  
rect = image.get_rect()   
counter_x = 1
counter_y = 1 
# keep rotating the rectangle until running is set to False  
running = True  
#while running:
for i, x in enumerate(x_mpc):
    y = y_mpc[i]
    phi = phi_mpc[i]
    rect.center = (x, y)  
    # set FPS  
    clock.tick(FPS)  
    # clear the screen every time before drawing new objects  
    screen.fill(BLACK)  
    # check for the exit  
    for event in py.event.get():  
        if event.type == py.QUIT:  
            running = False  

    # making a copy of the old center of the rectangle  
    old_center = rect.center


    # defining angle of the rotation  
    #rot = (rot + rot_speed) % 360 


    # rotating the orignal image  
    new_image = py.transform.rotate(image_orig , phi)  
    rect = new_image.get_rect()  
    # set the rotated rectangle to the old center  
    rect.center = old_center  
    # drawing the rotated rectangle to the screen  
    screen.blit(new_image , rect)  
    counter_x +=1
    counter_y +=1
    py.display.flip()  
     

py.quit()