import cv2
from fpdf import FPDF
import numpy as np
import os
import sys

circleDiameter = 50
numOfCircles = 9


################################################################################
####################### DO NOT EDIT BELOW THIS LINE! ###########################
################################################################################


numberPerRow = (210 / (circleDiameter + (circleDiameter / 5)))
numberPerColum = (297 / (circleDiameter + (circleDiameter / 5)))

def gen_single_circle(outsideCircleRad, insideCircleRad):
    img = np.ones(((outsideCircleRad * 2) + 12, (outsideCircleRad * 2) + 12, 3), np.uint8)
    img[:, :] = (255, 255, 255)
    
    for i in range(outsideCircleRad, insideCircleRad, -2):
        cv2.circle(img, (outsideCircleRad + 6, outsideCircleRad + 6), i, (0, 0, 0), 2)
    return img

def gen_pdf():
    pdf = FPDF()
    pdf.add_page()
    xPos = 0
    yPos = 1    
    for i in range(1, numOfCircles + 1):
        increments = 200 / numOfCircles
        cv2.imwrite(str(i) + 'circle.png', gen_single_circle(500+100, ((increments * i)+100)))
        cv2.imshow('circle.png', gen_single_circle(500+100, ((increments * i)+100)))
        
        k = cv2.waitKey(200) & 0xFF
        xPos += 1
        x = (xPos * (circleDiameter + (circleDiameter / 5)))-circleDiameter
        y = (yPos * (circleDiameter + (circleDiameter / 5)))
        sys.stdout.write('(' + str(x) + ' , ' + str(y) + ') ')
        pdf.image(str(i) + 'circle.png', x, y, circleDiameter + 3, circleDiameter + 3, 'png')    
        
        os.remove(str(i) + 'circle.png');
        
        if xPos > numberPerRow-1:
            print ""
            yPos += 1
            xPos = 0
        if yPos > numberPerColum:
            if (i != (numOfCircles)):
                pdf.add_page()
                print "-------------------------------------"
                xPos = 0
                yPos = 1
    pdf.output('BETA.pdf', 'F')
        
gen_pdf()
#cv2.imshow('image', a);
#k = cv2.waitKey(1000) & 0xFF