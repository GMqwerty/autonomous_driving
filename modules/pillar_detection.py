import cv2
import numpy as np 
import os 

class PillarDetection:
    def __init__(self, colors={"red":{"h":(170, 5), "s":(40, 200), "v":(90, 160), "num":1}, 
                               "green":{"h":(65, 85), "s":(0, 255), "v":(0, 255), "num":-1}}, cnf="cnf.txt"):
        self.cnf = cnf
        self.colors = colors

    def get_color_mask(self, color, image):
        hsv_range = self.colors[color]
        imgHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        if hsv_range["h"][0] < hsv_range["h"][1]:
            lower = np.array([hsv_range["h"][0], hsv_range["s"][0], hsv_range["v"][0]])
            upper = np.array([hsv_range["h"][1], hsv_range["s"][1], hsv_range["v"][1]])
            return cv2.inRange(imgHSV,lower,upper)
        else:
            lower = np.array([hsv_range["h"][0], hsv_range["s"][0], hsv_range["v"][0]])
            upper = np.array([179, hsv_range["s"][1], hsv_range["v"][1]])
            im1 = cv2.inRange(imgHSV,lower,upper)
            lower = np.array([0, hsv_range["s"][0], hsv_range["v"][0]])
            upper = np.array([hsv_range["h"][1], hsv_range["s"][1], hsv_range["v"][1]])
            im2 = cv2.inRange(imgHSV,lower,upper)
            im_or = cv2.bitwise_or(im1, im2)
            return im_or        
    
    def get_contours(self, image, mask, minArea=1000, sort= True, filter= 0,drawCon=True):
        """
        Finds Contours in an image

        :param img: Image on which we want to draw
        :param imgPre: Image on which we want to find contours
        :param minArea: Minimum Area to detect as valid contour
        :param sort: True will sort the contours by area (biggest first) 
        :param filter: Filters based on the corner points e.g. 4 = Rectangle or square
        :param drawCon: draw contours boolean
        :return: Foudn contours with [contours, Area, BoundingBox]
        """
        conFound = []
        imgContours = image.copy()
        # imgGray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        # imgBlur = cv2.GaussianBlur(imgGray,(blur,blur),1)
        # imgCanny = cv2.Canny(imgBlur,cannyThres[0],cannyThres[1])
        # kernel = np.ones((5,5), np.uint8)
        # imgDia = cv2.dilate(imgCanny, kernel, iterations =dia)
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for cnt in contours: 
            area = cv2.contourArea(cnt)
            if area > minArea:
                peri = cv2.arcLength(cnt,True)
                approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
                #print(len(approx))
                if len(approx) == filter or filter == 0:
                    if drawCon:cv2.drawContours(imgContours,cnt,-1, (255,0,255), 3)
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.rectangle(imgContours, (x,y), (x+w,y+h), (0,255,0),2)
                    cv2.circle(imgContours, (x+(w//2),y+(h//2)),5, (0,255,0), cv2.FILLED)
                    conFound.append([cnt,area,[x,y,w,h]])
        
        if sort: 
            conFound = sorted(conFound, key = lambda x:x[1], reverse = True)

        return imgContours, conFound
    
    def detect_pillars(self, image, error, colors=["red", "green"], line=True):
        hmax = -1
        x = y = w = h = cx = cy = 0
        pillar_found = False
        pillar_info = {"color":None, "height":0, "cx":0, "cy":0, "cerror":0}
        for color in colors:
            color_mask = self.get_color_mask(color, image)
            img_contours, con_found = self.get_contours(image, color_mask)
            if len(con_found)!= 0:
                pillar_found = True
                x,y,w,h = con_found[0][2]
                if h > hmax:
                    hmax = h
                    cx = x+(w//2)
                    cy = y+(h//2)
                    # print("x,y,w,h, cx, cy, color:", x,y,w,h, cx, cy, color)
                    hi, wi, ci = image.shape
                    ### Pid
                    error = int(((cx - wi//2) * 100) / (wi/2))
                    pillar_info["color"] = self.colors[color]["num"]
                    pillar_info["cx"] = cx
                    pillar_info["cy"] = cy
                    pillar_info["height"] = h
                    pillar_info["cerror"] = error

        
        if pillar_found and line:
            cv2.line(image, (wi//2,cy),(cx,cy),(255,0,255),2)

        return image, pillar_info
    
def main():
    error = 0
    pillar_detection = PillarDetection()
    folder = "Resources"
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
    if images:
        for image in images:
            result_image, pillar_info = pillar_detection.detect_pillars(image, colors=["green", "red"], line=True, error=error)
            cv2.imshow("Image",img)
            # cv2.imshow("Image Color",imgColor)
            cv2.imshow("Image Contours",result_image)
            print("Pillar:", pillar_info)


            if cv2.waitKey(0) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    main()