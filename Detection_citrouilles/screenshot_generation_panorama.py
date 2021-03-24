import pyautogui
from time import time
import shutil
import cv2
import os

#On récupère l'image de la camera
cam = cv2.VideoCapture(0)

verif = False
img_counter = 1
count = 0
files=[]

#On récupère le temps présent dans previous
previous = time()
delta = 0

#On prend  photos
while count <= 9:

    #On récupère le temps actuel
    current = time()
    #On fait la différence entre le temps qui s'est ecoule
    delta += current - previous
    previous = current

    #Si 2 secondes sont passes
    if delta > 2:
        count = count + 1
        #Enregistrement de image prise a cet instant par la camera
        image = 'panorama'+str(count)+'.jpg'
        print("enregistrement de" + image)
        cv2.imwrite(image, img)
        files.append(image)
        #Reset du compteur de temps
        delta = 0

    #On affiche l'image en temps reele donnee par la camera, pour verifier si tout se passe bien
    _, img = cam.read()
    cv2.imshow("Frame", img)
    
#On suprime les anciennes images contenus dans le dossier, et on les remplace par les nouvelles
for file in files:
    os.remove('/home/domi/Images/1/'+file)
    shutil.move(file,'/home/domi/Images/1')

######################
#CREATION DU PANORAMA#
######################

#On ouvre le dossier Images dans lequel se trouve nos images
mainFolder = 'Images'
myFolders = os.listdir(mainFolder)
print(myFolders)

#Pour chaque dossier dans le dossier (on en a qu'un seul par default, mais on pourrait imaginer en avoir plusieurs)
for folder in myFolders:
    #On ouvre chaque dossier dans le dossier principal
    path = mainFolder +'/'+ folder
    images = []
    myList = os.listdir(path)

    #On place les images contenus dans ce dossier dans une liste d'images
    for imgN in myList:
        curImg = cv2.imread(f'{path}/{imgN}')
        images.append(curImg)

    #On cree le panorama avec les images
    stitcher = cv2.createStitcher()
    (status,result) = stitcher.stitch(images)
    if (status == cv2.STITCHER_OK):
        print('Panorama genere')
        #cv2.resize(result, (200, 200))
        print("enregistrement")
        cv2.imwrite('panorama.jpg', result)
        print("affichage")
        cv2.imshow("panorama",result)
        cv2.waitKey(1)
    else:
        print('La generation a echoue')
        exit()

#Si l'utilisateur appui sur une touche tout se ferme
cv2.waitKey(0)
cam.release()
cv2.destroyAllWindows()