import cv2
import numpy as np
from collections import deque

# La taille du buffer qu'on va appliquer
buffer_size = 16
pts = deque(maxlen=buffer_size)

#Les deux bornes de couleurs HSV entre lesquelle le orange des citrouilles est compris
ORANGE_MIN = np.array([0, 151, 135],np.uint8)
ORANGE_MAX = np.array([25, 255, 255],np.uint8)

#On recupere une image (generee par un autre programme de creation de panoramas)
imgOriginal = cv2.imread ("DSC_0621.JPG")

#On "floute" l'image, elle est enregistree sous le nom "blurred.png"
blurred = cv2.GaussianBlur(imgOriginal, (11,11), 0)
cv2.imwrite("blurred.png", blurred)

#On change les couleurs de l'image en HSV, elle est enregistrée sous le nom "hsv.png"
hsv = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2HSV)
cv2.imwrite("hsv.png", hsv)
        
#On applique le masque orange sur l'image HSV, elle est enregistrée sous le nom "mask.png"
mask = cv2.inRange(hsv, ORANGE_MIN, ORANGE_MAX)
cv2.imwrite("mask.png", mask)

#On detecte tous les contours sur l'image
contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]
center = None

#Pour chacun des coutours detectes (ce qui comprends les parasites):
for c in contours:

    #On recupere l'aire du contours
    area = cv2.contourArea(c)

    #Si l'aire est assez grande (on élimine les parasites, pour ne garder que les contours de citrouilles) :
    if area > 40000:

        #On determine un rectangle anglobant le contours
        rect = cv2.minAreaRect(c)

        #On récupères les coordonées du rectangle (La position de son centre, sa taille, et son orientation)
        ((x,y), (width, height), rotation) = rect
        #On print ces coordonées
        s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
        print(s)

        #On créer une box du rectangle pour pouvoir l'afficher sur l'image
        box = cv2.boxPoints(rect)
        box = np.int64(box)

        #On trouve le centre du rectangle
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        #On dessine les contours
        cv2.drawContours(imgOriginal, [box], 0, (255, 255, 255), 5)

        #On dessine le point au centre du rectangle
        cv2.circle(imgOriginal, center, 30, (255, 255, 255), -1)

        #On ecrit dans le coin de l'image les coordonées du rectangle
        cv2.putText(imgOriginal, s, (25, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255), 2)


pts.appendleft(center)
for i in range(1, len(pts)):

    if pts[i - 1] is None or pts[i] is None: continue

    cv2.line(imgOriginal, pts[i - 1], pts[i], (0, 255, 0), 3)

#On enregistre l'image avec les citrouilles detectées (rectangle autours des citrouilles) sous le nom "citrouille_detected.png"
cv2.imwrite("citrouille_detected.png", imgOriginal)