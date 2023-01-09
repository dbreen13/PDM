import numpy as np
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

car_color = (255, 0, 0)
car_size = (20, 30)

#Test MPC-values
x_mpc = (0 , 20, 40, 60, 80, 100, 120, 140, 160, 180, 200)
y_mpc = (0, 10, 30, 60, 100, 150, 210, 280, 360, 450, 550)
phi_mpc = [0,  5.2359878,  10.4719755,  15.7079633,  20.943951 , 26.1799388,  31.4159265,  36.6519143,  41.887902 ,  47.1238898, 52.3598776]

# define a surface (RECTANGLE)  
image_orig = py.Surface(car_size)  
# for making transparent background while rotating an image  
image_orig.set_colorkey(BLACK)  
# fill the rectangle / surface with green color  
image_orig.fill(car_color)  
# creating a copy of orignal image for smooth rotation  
image = image_orig.copy()  
image.set_colorkey(BLACK)   

# Define the Car class
class Car:
    def __init__(self, x, y, v, phi, a, delta):
        self.x = x # x-position of the vehicle
        self.y = y # y-position of the vehicle
        self.v = v # velocity of the vehicle
        self.phi = phi # orientation angle of the vehicle
        self.acc = a # acceleration of the vehicle
        self.delta = delta # orientation angle of the front wheels of the vehicle
        self.m = 0.5
        self.L = 1
        self.I = (1/12)*self.m*self.L**2
        self.a = self.L/2
        self.b = self.L - self.a

    def update(self):
        # define rect for placing the rectangle at the desired position  
        rect = image.get_rect() 
        # Update the position and velocity of the car
        rect.center = (x, y)
        
        # Draw the car on the screen
        #car_rect = py.Rect(car.x, car.y, car_size[0], car_size[1])
        #py.draw.rect(screen, car_color, car_rect)

        #making a copy of the old center of the rectangle  
        old_center = rect.center

        # rotating the orignal image  
        new_image = py.transform.rotate(image_orig , phi)  
        rect = new_image.get_rect()

        # set the rotated rectangle to the old center  
        rect.center = old_center

        # drawing the rotated rectangle to the screen  
        screen.blit(new_image , rect)

        # Update the Pygame display
        py.display.flip()  



# Main loop
running = True
for i, x in enumerate(x_mpc):
    x = x_mpc[i]
    y = y_mpc[i]
    phi = phi_mpc[i] 
    # set FPS  
    clock.tick(FPS)  
    # clear the screen every time before drawing new objects  
    screen.fill(BLACK)  
    # check for the exit  
    for event in py.event.get():  
        if event.type == py.QUIT:  
            running = False

    # Create an instance of the Car class
    car = Car(x, y, 0, phi, 0, 0)
    #car = Car(x_mpc,y_mpc,v_mpc,phi_mpc,a_mpc,delta_mpc) # The state-vector values that come from the MPC

    # Update the car's position and orientation
    car.update()



py.quit()