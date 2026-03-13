import cv2

if __name__ == "__main__":
    camera = cv2.VideoCapture(0)
    success, frame = camera.read()
    cv2.imwrite('/home/pi/PickPro/python/task/VV_file_final/cap_photo/' + 'hello' + '.jpg', frame)
    print(frame.shape)
    '''
    while camera.isOpened():
        success, frame = camera.read()
        if not success: break
        cv2.imwrite('/home/pi/PickPro/python/task/VV_file_final/cap_photo/' + 'hello' + '.jpg', frame)
        print(frame.shape)
    '''
        