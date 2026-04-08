import gyroscope

if __name__ == "__main__":
    gyro = gyroscope.MPU6050()
    while True:
        neigung = gyro.get_x_rotation()
        print(neigung)