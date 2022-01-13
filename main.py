import lidar
import vehicle

scanner = lidar.scanner()
car = vehicle.car()

try:
    scanner.start()
    car.run(scanner.scan_data)
    # scanner.join()                 # waits on this position until Scanner-Thread finished
except KeyboardInterrupt:
    print('Keyboard Interrupt')
finally:
    scanner.stop()
    car.stop()

