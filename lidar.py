import threading
from adafruit_rplidar import RPLidar
from math import floor

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)


class scanner(threading.Thread):
    MaxDistance = 3000
    scan_data = [0] * 360

    def __init__(self, *args, **kwargs):
        super(scanner, self).__init__(*args, **kwargs)  # threading.Thread.__init__(self)
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        print("Lidar:" + str(lidar.info))
        for scan in lidar.iter_scans():
            if not self.stopped():
                for (_, angle, distance) in scan:
                    self.scan_data[min([359, floor(angle)])] = min(distance, self.MaxDistance)
            else:
                print("Lidar stopped")
                lidar.stop()
                lidar.disconnect()
                break






