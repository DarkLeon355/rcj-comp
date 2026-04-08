import smbus
import math
import time

class MPU6050:
    def __init__(self):
        # I2C und Register Setup
        self.Device_Address = 0x68
        self.PWR_MGMT_1   = 0x6B
        self.SMPLRT_DIV   = 0x19
        self.CONFIG       = 0x1A
        self.GYRO_CONFIG  = 0x1B
        self.INT_ENABLE   = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_ZOUT_H  = 0x47
        self.GYRO_SENS = 16.4 
        self.gyro_offset = 0.0

        try:
            self.bus = smbus.SMBus(1)
            self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)
            self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)
            self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)
            self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)
            self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)
            
            print("MPU6050 verbunden.")
            
            # 1. Countdown (Damit du Zeit hast, loszulassen)
            print("Start in 3 Sekunden...")
            for i in range(3, 0, -1):
                print(f"{i}...")
                time.sleep(1)
            
            # 2. Kalibrierung (Offset messen)
            # Der Roboter sollte hier trotzdem möglichst still stehen!
            self.calibrate_gyro() 
            
            # HIER wurde der "wait_until_stable" Check entfernt.
            # Der Code geht jetzt sofort weiter zur main.py

        except Exception as e:
            print(f"Fehler beim Starten des MPU6050: {e}")

    def read_raw_data(self, addr):
        try:
            high = self.bus.read_byte_data(self.Device_Address, addr)
            low = self.bus.read_byte_data(self.Device_Address, addr + 1)
            value = (high << 8) | low
            if value > 32767: value -= 65536
            return value
        except: return 0

    def calibrate_gyro(self):
        """Misst den Offset."""
        print("Kalibriere Gyro (kurz)...")
        samples = 100
        sum_z = 0
        for i in range(samples):
            sum_z += self.read_raw_data(self.GYRO_ZOUT_H)
            time.sleep(0.01) 
        self.gyro_offset = (sum_z / samples) / self.GYRO_SENS
        print(f"Offset gesetzt: {self.gyro_offset:.2f} °/s")
        print(">>> START! <<<")

    def get_gyro_z(self):
        raw_z = self.read_raw_data(self.GYRO_ZOUT_H)
        return (raw_z / self.GYRO_SENS) - self.gyro_offset

    def get_x_rotation(self):
        """Calculate pitch angle (uphill/downhill tilt) in degrees."""
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)
        
        # For pitch (rotation around Y-axis = uphill/downhill):
        # Use atan2(acc_x, sqrt(acc_y^2 + acc_z^2))
        dist = math.sqrt((acc_y * acc_y) + (acc_z * acc_z))
        if dist == 0: return 0
        return math.degrees(math.atan2(acc_x, dist))



if __name__ == "__main__":
    gyro = MPU6050()
    while True:
        print(gyro.get_x_rotation())
        time.sleep(0.1)