import smbus
import math
import time

class MPU6050:
    """MPU6050 gyroscope/accelerometer helper for Raspberry Pi I2C."""

    def __init__(self) -> None:
        """Initialize sensor registers and perform startup calibration."""
        #─────────────────────────────────────────────────────────────
        # DEFINITIONS:
        #─────────────────────────────────────────────────────────────
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
        #─────────────────────────────────────────────────────────────

        #─────────────────────────────────────────────────────────────
        # SETUP:
        #─────────────────────────────────────────────────────────────
        try:
            self.bus = smbus.SMBus(1)
            self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)
            self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)
            self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)
            self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)
            self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)
            
            print("MPU6050 connected.")
            
            # calib countdown
            print("Start in 3 seconds...")
            for i in range(3, 0, -1):
                print(f"{i}...")
                time.sleep(1)
            
            # calib gyro
            self.calibrate_gyro() 
            
        except:
            raise Exception("Error during MPU6050 startup")
        #─────────────────────────────────────────────────────────────

    def read_raw_data(self, addr: int) -> int:
        """Read a signed 16-bit value from the given register address.

        Args:
            addr: Register start address.

        Returns:
            Signed 16-bit sensor value, or 0 if reading fails.
        """
        try:
            high = self.bus.read_byte_data(self.Device_Address, addr)
            low = self.bus.read_byte_data(self.Device_Address, addr + 1)
            value = (high << 8) | low
            if value > 32767: value -= 65536
            return value
        except: return 0

    def calibrate_gyro(self) -> None:
        """Measure and store gyro z-axis offset while the robot is still."""
        print("Calibrate Gyro...")
        samples = 100
        sum_z = 0
        for i in range(samples):
            sum_z += self.read_raw_data(self.GYRO_ZOUT_H)
            time.sleep(0.01) 
        self.gyro_offset = (sum_z / samples) / self.GYRO_SENS
        print(f"Offset set: {self.gyro_offset:.2f} °/s")

    def get_gyro_z(self) -> float:
        """Return calibrated angular velocity around z-axis in degrees/s."""
        raw_z = self.read_raw_data(self.GYRO_ZOUT_H)
        return (raw_z / self.GYRO_SENS) - self.gyro_offset

    def get_x_rotation(self) -> float:
        """Calculate pitch angle (uphill/downhill tilt) in degrees."""
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)
        
        dist = math.sqrt((acc_y * acc_y) + (acc_z * acc_z))
        if dist == 0: return 0
        return math.degrees(math.atan2(acc_x, dist))


if __name__ == "__main__":
    gyro = MPU6050()
    while True:
        print(gyro.get_x_rotation())
        time.sleep(0.1)