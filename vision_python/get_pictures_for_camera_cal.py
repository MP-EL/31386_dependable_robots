import cv2
import matplotlib.pyplot as plt
import os


lenght = [15,30,45,60,75,90,105,120,135,150]

for index,value in enumerate(lenght):
    cap = cv2.VideoCapture(2) # video capture source camera (Here webcam of laptop) 

    input(f"current measure value: {value} \n press anything to continue")


    ret,frame = cap.read() # return a single frame in variable `frame`
    cv2.imwrite(f'{value}.png',frame)

    cap.release()
