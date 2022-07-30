from Img_process import *

def main():
    cap = cv.VideoCapture(1)
    frame_width = 1024
    frame_height = 768

    cap.set(cv.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, frame_height)
    if not cap.isOpened():
        print('can not open camera')
        exit()

    # loop
    while(cap.isOpened()):
        ret, frame = cap.read() # read a frame from video/camera
        if ret == True:
            iP_output = imagePreprocess(frame)
            
            contours,hierarchy = cv.findContours(iP_output, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

            hierarchy = hierarchy[0] # get the actual inner list of hierarchy descriptions

            # For each contour, find the bounding rectangle and draw it
            for component in zip(contours, hierarchy):
                currentContour = component[0]
                currentHierarchy = component[1]
                if cv.contourArea(currentContour) > 2000:
                    x,y,w,h = cv.boundingRect(currentContour)
                    if currentHierarchy[2] < 0:
                        # these are the innermost child components
                        cv.drawContours(frame, [currentContour], 0, (0,255,0), 2)
                        print('inner contour area', cv.contourArea(currentContour))
                    elif currentHierarchy[3] < 0:
                        # these are the outermost parent components
                        cv.drawContours(frame, [currentContour], 0, (0,255,0), 2)
                        print('outer contour area', cv.contourArea(currentContour))
            cv.imshow('out', frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()