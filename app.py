from gydar import *

if __name__ == "__main__":
    x = Gydar()
    print("Using ports: {} & {}".format(x.gyro_port, x.lidar_port))
    x.connect()
    while True:
        print(x.raw_lidar_output)
        print(x.raw_gyro_output)