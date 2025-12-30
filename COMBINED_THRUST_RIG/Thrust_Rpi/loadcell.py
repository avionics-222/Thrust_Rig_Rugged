import time
import statistics
from gpiozero import DigitalInputDevice, DigitalOutputDevice
import sys

dout = 22
sck = 27

class HX711:
    def __init__(self, dout_pin=dout, sck_pin=sck):
        """Initialize HX711 using gpiozero"""
        try:
            self.dout = DigitalInputDevice(dout_pin)
            self.sck = DigitalOutputDevice(sck_pin, initial_value=False)
        except Exception as e:
            print(f"Error initializing GPIO: {e}")
            sys.exit(1)

        self.offset_3v3_100 = 0.6237          # FOR 120KG: 0.6625  || FOR 150KG: 0.6237
        self.offset = 0 
        self.last_valid_reading = 0
        time.sleep(0.2)  # Reduced startup delay

    def read(self):
        """Optimized HX711 raw data reading with reduced delay"""
        timeout_start = time.time()
        while self.dout.value == 1:  # Wait for HX711 to be ready
            if time.time() - timeout_start > 0.1:  # 100ms timeout
                return None

        data = 0 
        for _ in range(24):
            self.sck.on() 
            self.sck.off()
            data = (data << 1) | self.dout.value  # Faster bit shifting

        self.sck.on()
        self.sck.off()  # 25th clock pulse

        if data & 0x800000:
            data -= 0x1000000  # Convert 2's complement

        return None if data == 0 or data == -1 else data

    def read_average(self, num_readings=3, max_retries=5):
        """Optimized function to take multiple readings and return median-filtered average"""
        readings = []
        retries = 0

        while len(readings) < num_readings and retries < max_retries:
            reading = self.read()
            if reading is not None:
                readings.append(reading)
            else:
                retries += 1
            time.sleep(0.005)  # Reduced sleep time

        if not readings:
            return self.last_valid_reading if self.last_valid_reading else 0

        return statistics.median(readings)  # Faster noise filtering

    def zero(self, num_readings=14):
        """Optimized tare function using median filtering"""
        print("Zeroing scale...")
        self.offset = self.read_average(num_readings)
        print(f"Zero offset: {self.offset}")
        return self.offset

    def get_weight(self, num_readings=3):
        """Get weight using voltage-based calibration for 3.3V"""
        reading = self.read_average(num_readings)
        raw_value = reading - self.offset
        out_volt = raw_value * 20 / 8388607
        weight = out_volt * 150 * 1000 / 6.6 * self.offset_3v3_100  # 6.6 = 3.3V * 2mV
        return weight, raw_value  # Convert to kg

    def reset_hx711(self):
        """Reset the HX711 chip by toggling the clock line"""
        self.sck.on()
        time.sleep(0.06)
        self.sck.off()
        time.sleep(0.01)

    def cleanup(self):
        """Cleanup resources (gpiozero handles this automatically)"""
        pass

def main():
    """Main function to demonstrate optimized HX711 readings"""
    hx = None
    try:
        hx = HX711(dout_pin=dout, sck_pin=sck)
        print(f"Using voltage-based calibration with offset_3v3_100: {hx.offset_3v3_100}")

        hx.zero()
        print("Scale ready! Starting weight measurements...")
        print("Press Ctrl+C to exit")

        failure_count = 0

        while True:
            try:
                weight, raw_value = hx.get_weight(num_readings=3)  # Reduced readings
                print(f"Raw value: {raw_value:.2f} | Weight: {weight:.2f} g")
                failure_count = 0  # Reset on success
            except Exception as e:
                print(f"Error: {e}")
                failure_count += 1
                if failure_count >= 3:  # Reduced reset threshold
                    print("Resetting HX711...")
                    hx.reset_hx711()
                    failure_count = 0

            time.sleep(0.25)  # Faster updates

    except KeyboardInterrupt:
        print("\nExiting program")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if hx:
            hx.cleanup()

if __name__ == "__main__":
    main()
