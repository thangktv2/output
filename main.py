from imgProcess import *

def main():
    cap = cv.VideoCapture(0)
    setCapSize(cap, 1024, 768)
    result = cap.isOpened()
    if result is False:
        print('Open camera failed')
        exit()
    setGlobalValue(cap,0.04)
    while(result):
        ret, frame = cap.read() # read a frame from video/camera
        if ret == True:
            iP_output = imagePreprocess(frame)
            gIC_output = getItemCoordinate(iP_output)
            if gIC_output[0] == 1:
                frame = cv.rectangle(frame, (gIC_output[1] - 5, gIC_output[2] - 5), (gIC_output[1] + gIC_output[3] + 5, gIC_output[2] + gIC_output[4] + 5), (0, 0, 255), 2)
            cv.imshow('out', frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()