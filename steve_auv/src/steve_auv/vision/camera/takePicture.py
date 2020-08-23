
from picamera import PiCamera
from time import sleep
camera = PiCamera()

camera.start_preview()
sleep(5)
i=1
while True:
    sleep(5)
    camera.capture('/home/pi/ScubaSteve/steve_auv/src/steve_auv/transducers/camera/images/image%s.jpg' % i)
    i+=1
camera.stop_preview()

