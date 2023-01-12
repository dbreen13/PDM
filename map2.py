import numpy as np
import pygame as py
from map_environment.obstacles import Rectangle
from map_environment.obstacles import Circle

#Map environment 2
if __name__ == '__main__':
    # Program variables
    v_shelf_1 = Rectangle(np.array([80, 50]), np.array([16, 200]))
    v_shelf_2 = Rectangle(np.array([120, 50]), np.array([16, 200]))
    v_shelf_3 = Rectangle(np.array([160, 50]), np.array([16, 200]))
    v_shelf_4 = Rectangle(np.array([200, 50]), np.array([16, 200]))
    v_shelf_5 = Rectangle(np.array([240, 50]), np.array([16, 200]))

    v_shelf_6 = Rectangle(np.array([500, 50]), np.array([16, 200]))
    v_shelf_7 = Rectangle(np.array([540, 50]), np.array([16, 200]))
    v_shelf_8 = Rectangle(np.array([580, 50]), np.array([16, 200]))
    v_shelf_9 = Rectangle(np.array([620, 50]), np.array([16, 200]))
    v_shelf_10 = Rectangle(np.array([660, 50]), np.array([16, 200]))

    v_shelf_11 = Rectangle(np.array([280, 300]), np.array([16, 200]))
    v_shelf_12 = Rectangle(np.array([320, 300]), np.array([16, 200]))
    v_shelf_13 = Rectangle(np.array([360, 300]), np.array([16, 200]))
    v_shelf_14 = Rectangle(np.array([400, 300]), np.array([16, 200]))
    v_shelf_15 = Rectangle(np.array([440, 300]), np.array([16, 200]))

    v_shelf_16 = Rectangle(np.array([500, 300]), np.array([16, 200]))
    v_shelf_17 = Rectangle(np.array([540, 300]), np.array([16, 200]))
    v_shelf_18 = Rectangle(np.array([580, 300]), np.array([16, 200]))
    v_shelf_19 = Rectangle(np.array([620, 300]), np.array([16, 200]))
    v_shelf_20 = Rectangle(np.array([660, 300]), np.array([16, 200]))

    v_shelf_21 = Rectangle(np.array([80, 530]), np.array([16, 200]))
    v_shelf_22 = Rectangle(np.array([120, 530]), np.array([16, 200]))
    v_shelf_23 = Rectangle(np.array([160, 530]), np.array([16, 200]))
    v_shelf_24 = Rectangle(np.array([200, 530]), np.array([16, 200]))
    v_shelf_25 = Rectangle(np.array([240, 530]), np.array([16, 200]))


    v_shelf = (v_shelf_1, v_shelf_2, v_shelf_3, v_shelf_4, v_shelf_5, v_shelf_6, v_shelf_7, v_shelf_8, v_shelf_9, v_shelf_10, 
    v_shelf_11, v_shelf_12, v_shelf_13, v_shelf_14, v_shelf_15, v_shelf_16, v_shelf_17, v_shelf_18, v_shelf_19, v_shelf_20,
    v_shelf_21, v_shelf_22, v_shelf_23, v_shelf_24, v_shelf_25)



    p_pillar_1 = Circle(np.array([50, 50]), 10)
    p_pillar_2 = Circle(np.array([50, 150]), 10)
    p_pillar_3 = Circle(np.array([50, 250]), 10)

    p_pillar_4 = Circle(np.array([50, 500]), 10)
    p_pillar_5 = Circle(np.array([50, 600]), 10)
    p_pillar_6 = Circle(np.array([50, 700]), 10)

    p_pillar_7 = Circle(np.array([750, 50]), 10)
    p_pillar_8 = Circle(np.array([750, 150]), 10)
    p_pillar_9 = Circle(np.array([750, 250]), 10)

    p_pillar_10 = Circle(np.array([750, 500]), 10)
    p_pillar_11 = Circle(np.array([750, 600]), 10)
    p_pillar_12 = Circle(np.array([750, 700]), 10)

    p_pillar_13 = Circle(np.array([480, 300]), 10)
    p_pillar_14 = Circle(np.array([480, 400]), 10)
    p_pillar_15 = Circle(np.array([480, 500]), 10)

    p_pillar = (p_pillar_1, p_pillar_2, p_pillar_3, p_pillar_4, p_pillar_5, p_pillar_6, p_pillar_7, p_pillar_8, p_pillar_9, p_pillar_10,
    p_pillar_11, p_pillar_12, p_pillar_13, p_pillar_14, p_pillar_15)



    h_shelf_1 = Rectangle(np.array([280, 60]), np.array([200, 16]))
    h_shelf_2 = Rectangle(np.array([280, 100]), np.array([200, 16]))
    h_shelf_3 = Rectangle(np.array([280, 140]), np.array([200, 16]))
    h_shelf_4 = Rectangle(np.array([280, 180]), np.array([200, 16]))
    h_shelf_5 = Rectangle(np.array([280, 220]), np.array([200, 16]))

    h_shelf_6 = Rectangle(np.array([50, 280]), np.array([200, 16]))
    h_shelf_7 = Rectangle(np.array([50, 320]), np.array([200, 16]))
    h_shelf_8 = Rectangle(np.array([50, 360]), np.array([200, 16]))
    h_shelf_9 = Rectangle(np.array([50, 400]), np.array([200, 16]))
    h_shelf_10 = Rectangle(np.array([50, 440]), np.array([200, 16]))

    h_shelf_11 = Rectangle(np.array([280, 540]), np.array([200, 16]))
    h_shelf_12 = Rectangle(np.array([280, 580]), np.array([200, 16]))
    h_shelf_13 = Rectangle(np.array([280, 620]), np.array([200, 16]))
    h_shelf_14 = Rectangle(np.array([280, 660]), np.array([200, 16]))
    h_shelf_15 = Rectangle(np.array([280, 700]), np.array([200, 16]))

    h_shelf_16 = Rectangle(np.array([500, 540]), np.array([200, 16]))
    h_shelf_17 = Rectangle(np.array([500, 580]), np.array([200, 16]))
    h_shelf_18 = Rectangle(np.array([500, 620]), np.array([200, 16]))
    h_shelf_19 = Rectangle(np.array([500, 660]), np.array([200, 16]))
    h_shelf_20 = Rectangle(np.array([500, 700]), np.array([200, 16]))


    h_shelf = (h_shelf_1, h_shelf_2, h_shelf_3, h_shelf_4, h_shelf_5, h_shelf_6, h_shelf_7, h_shelf_8, h_shelf_9, h_shelf_10,
    h_shelf_11, h_shelf_12, h_shelf_13, h_shelf_14, h_shelf_15, h_shelf_16, h_shelf_17, h_shelf_18, h_shelf_19, h_shelf_20)