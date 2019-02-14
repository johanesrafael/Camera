import picamera
camera = picamera.PiCamera()
camera.capture('mytest_image.jpg')
camera.close()
