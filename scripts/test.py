import cv2 as cv
from cvzone.ColorModule import ColorFinder
import cvzone

source = cv.VideoCapture(0) 
source.set(3,1280)
source.set(4,720)

hsvVals = {'hmin': 10, 'smin': 55, 'vmin': 215, 'hmax': 42, 'smax': 255, 'vmax': 255}

color_finder = ColorFinder(True)

while True:
    isTrue, frame = source.read() 

    if not isTrue:
        break 
    img_color, mask = color_finder.update(frame, hsvVals)

    stak = cvzone.stackImages([frame, img_color], 2, 0.5)

    cv.imshow('fa', stak)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
        



    # Release the video capture and close all windows
source.release()
cv.destroyAllWindows()