# serial_reader.py
import serial
import threading
class SerialReader:
    def __init__(self, port, baudrate, callback=None):
        self.port = port
        self.baudrate = baudrate
        self.callback = callback
        self.serial = None
        self.thread = None
        self.running = False

    def start(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            self.thread = threading.Thread(target=self.read_loop)
            self.thread.start()
            return f"[SerialReader] Started on {self.port} at {self.baudrate} baud."
        except Exception as e:
            print(f"[SerialReader] Error: {e}")
            return f"[SerialReader] Error: {e}"

    def stop(self):
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        return f"[SerialReader] Stopped reading from {self.port}."

    def read_loop(self):
        while self.running:
            try:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('L:'):
                    data = self.parse_line(line)
                    if data and self.callback:
                        self.callback(data)
            except Exception as e:
                print(f"[SerialReader] Error: {e}")

    def parse_line(self, line):
        try:
            # Expected format: L: 1,2,3,4,5,6
            line = line.replace('L:', '').strip()
            values = [float(v.strip()) for v in line.split(',')]
            if len(values) == 6:
                return {
                    'lc1': values[0]/10000,
                    'lc2': values[1]/10000,
                    'lc3': values[2]/10000,
                    'lc4': values[3]/10000,
                    'lc5': values[4]/10000,
                    'lc6': values[5]/10000
                }
        except:
            return None
