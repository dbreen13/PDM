import numpy as np
import pygame as py
from map_environment.obstacles import Rectangle
from map_environment.obstacles import Circle

#Map environment 1
if __name__ == '__main__':
    # Program variables
    v_shelf_1 = Rectangle(np.array([30, 50]), np.array([16, 200]))
    v_shelf_2 = Rectangle(np.array([70, 50]), np.array([16, 200]))
    v_shelf_3 = Rectangle(np.array([110, 50]), np.array([16, 200]))
    v_shelf_4 = Rectangle(np.array([150, 50]), np.array([16, 200]))
    v_shelf_5 = Rectangle(np.array([190, 50]), np.array([16, 200]))

    v_shelf_6 = Rectangle(np.array([300, 290]), np.array([16, 240]))
    v_shelf_7 = Rectangle(np.array([340, 290]), np.array([16, 240]))
    v_shelf_8 = Rectangle(np.array([380, 290]), np.array([16, 240]))
    v_shelf_9 = Rectangle(np.array([420, 290]), np.array([16, 240]))

    v_shelf_10 = Rectangle(np.array([90, 530]), np.array([16, 200]))
    v_shelf_11 = Rectangle(np.array([130, 530]), np.array([16, 200]))
    v_shelf_12 = Rectangle(np.array([170, 530]), np.array([16, 200]))
    v_shelf_13 = Rectangle(np.array([210, 530]), np.array([16, 200]))

    v_shelf_14 = Rectangle(np.array([500, 50]), np.array([16, 160]))
    v_shelf_15 = Rectangle(np.array([540, 50]), np.array([16, 160]))
    v_shelf_16 = Rectangle(np.array([580, 50]), np.array([16, 200]))

    v_shelf_17 = Rectangle(np.array([740, 300]), np.array([16, 200]))
    v_shelf_18 = Rectangle(np.array([780, 300]), np.array([16, 200]))
    v_shelf_19 = Rectangle(np.array([820, 300]), np.array([16, 200]))
    v_shelf_20 = Rectangle(np.array([860, 300]), np.array([16, 200]))

    v_shelf_21 = Rectangle(np.array([780, 530]), np.array([16, 200]))
    v_shelf_22 = Rectangle(np.array([820, 530]), np.array([16, 200]))


    v_shelf = (v_shelf_1, v_shelf_2, v_shelf_3, v_shelf_4, v_shelf_5, v_shelf_6, v_shelf_7, v_shelf_8, v_shelf_9, v_shelf_10, 
    v_shelf_11, v_shelf_12, v_shelf_13, v_shelf_14, v_shelf_15, v_shelf_16, v_shelf_17, v_shelf_18, v_shelf_19, v_shelf_20,
    v_shelf_21, v_shelf_22)



    p_pillar_1 = Circle(np.array([320, 260]), 10)
    p_pillar_2 = Circle(np.array([420, 260]), 10)
    p_pillar_3 = Circle(np.array([520, 260]), 10)

    p_pillar_4 = Circle(np.array([50, 550]), 10)
    p_pillar_5 = Circle(np.array([50, 650]), 10)
    p_pillar_6 = Circle(np.array([50, 750]), 10)

    p_pillar_7 = Circle(np.array([880, 50]), 10)
    p_pillar_8 = Circle(np.array([880, 150]), 10)
    p_pillar_9 = Circle(np.array([880, 250]), 10)

    p_pillar_10 = Circle(np.array([520, 550]), 10)
    p_pillar_11 = Circle(np.array([620, 550]), 10)
    p_pillar_12 = Circle(np.array([720, 550]), 10)

    p_pillar_13 = Circle(np.array([880, 550]), 10)
    p_pillar_14 = Circle(np.array([880, 650]), 10)
    p_pillar_15 = Circle(np.array([880, 750]), 10)

    p_pillar = (p_pillar_1, p_pillar_2, p_pillar_3, p_pillar_4, p_pillar_5, p_pillar_6, p_pillar_7, p_pillar_8, p_pillar_9, p_pillar_10,
    p_pillar_11, p_pillar_12, p_pillar_13, p_pillar_14, p_pillar_15)



    h_shelf_1 = Rectangle(np.array([250, 50]), np.array([200, 16]))
    h_shelf_2 = Rectangle(np.array([250, 90]), np.array([200, 16]))
    h_shelf_3 = Rectangle(np.array([250, 130]), np.array([200, 16]))
    h_shelf_4 = Rectangle(np.array([250, 170]), np.array([200, 16]))
    h_shelf_5 = Rectangle(np.array([250, 210]), np.array([200, 16]))

    h_shelf_6 = Rectangle(np.array([30, 300]), np.array([240, 16]))
    h_shelf_7 = Rectangle(np.array([30, 340]), np.array([240, 16]))
    h_shelf_8 = Rectangle(np.array([30, 380]), np.array([240, 16]))
    h_shelf_9 = Rectangle(np.array([30, 420]), np.array([240, 16]))
    h_shelf_10 = Rectangle(np.array([30, 460]), np.array([240, 16]))
    #h_shelf_11 = Rectangle(np.array([30, 480]), np.array([240, 16]))

    h_shelf_12 = Rectangle(np.array([620, 50]), np.array([200, 16]))
    h_shelf_13 = Rectangle(np.array([620, 90]), np.array([200, 16]))
    h_shelf_14 = Rectangle(np.array([620, 130]), np.array([200, 16]))
    h_shelf_15 = Rectangle(np.array([620, 170]), np.array([200, 16]))
    h_shelf_16 = Rectangle(np.array([620, 210]), np.array([200, 16]))

    h_shelf_17 = Rectangle(np.array([480, 320]), np.array([200, 16]))
    h_shelf_18 = Rectangle(np.array([480, 360]), np.array([200, 16]))
    h_shelf_19 = Rectangle(np.array([480, 400]), np.array([200, 16]))
    h_shelf_20 = Rectangle(np.array([480, 440]), np.array([200, 16]))
    h_shelf_21 = Rectangle(np.array([480, 480]), np.array([200, 16]))

    h_shelf_22 = Rectangle(np.array([250, 560]), np.array([200, 16]))
    h_shelf_23 = Rectangle(np.array([250, 600]), np.array([200, 16]))
    h_shelf_24 = Rectangle(np.array([250, 640]), np.array([200, 16]))
    h_shelf_25 = Rectangle(np.array([250, 680]), np.array([200, 16]))
    h_shelf_26 = Rectangle(np.array([250, 720]), np.array([200, 16]))

    h_shelf_27 = Rectangle(np.array([500, 600]), np.array([240, 16]))
    h_shelf_28 = Rectangle(np.array([500, 640]), np.array([240, 16]))
    h_shelf_29 = Rectangle(np.array([500, 680]), np.array([240, 16]))
    h_shelf_30 = Rectangle(np.array([500, 720]), np.array([240, 16]))


    h_shelf = (h_shelf_1, h_shelf_2, h_shelf_3, h_shelf_4, h_shelf_5, h_shelf_6, h_shelf_7, h_shelf_8, h_shelf_9, h_shelf_10,
     h_shelf_12, h_shelf_13, h_shelf_14, h_shelf_15, h_shelf_16, h_shelf_17, h_shelf_18, h_shelf_19, h_shelf_20,
     h_shelf_21, h_shelf_22, h_shelf_23, h_shelf_24, h_shelf_25, h_shelf_26, h_shelf_27, h_shelf_28, h_shelf_29, h_shelf_30)