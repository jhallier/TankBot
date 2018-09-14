from picamera import PiCamera
from time import sleep

camera = PiCamera()


def capture_preview(time):
    camera.start_preview()
    sleep(time)
    camera.stop_preview()

def capture_n(folder, total_nr, freq):
    interval = 1 / freq
    for i in range(total_nr):
        filename = folder+"/image"+str(i)+".jpg"
        camera.capture(filename)
        print("Image {0} captured".format(filename))
        sleep(interval)


folder = "ImageCapture"

capture_n(folder, 100, 2)

