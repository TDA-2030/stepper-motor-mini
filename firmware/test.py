from serial import Serial, serialutil
from serial.threaded import ReaderThread, Protocol,Packetizer
import time
from queue import Queue, Empty

class SerialReaderProtocolRaw(Packetizer):

    def __init__(self) -> None:
        super().__init__()
        Packetizer.TERMINATOR = b'\x6B'
        self.recv_queue:Queue = None

    def connection_made(self, transport: ReaderThread) -> None:
        """Called when reader thread is started"""
        super().connection_made(transport)
        print("COM Connected, ready to receive data...")

    # def data_received(self, data: bytes) -> None:
    #     """Called with snippets received from the serial port"""
    #     print(f"recv:{data}")
    #     self.recv_queue.put(data)
    def handle_packet(self, packet: bytes) -> None:
        packet = bytes(packet)
        # print(f"recv:{packet}")
        self.recv_queue.put(packet)

    def connection_lost(self, exc: BaseException) -> None:
        super().connection_lost(exc)
        print("COM disconnected")


class COM:

    def __init__(self, port, baud):
        self.port = port
        self.baud = int(baud)
        self.open_com = None
        self.recv_queue = Queue(10)
        
    def open(self):
        # Initiate serial port
        print(f"COM port:{self.port} baud:{self.baud}")
        self.serial_port = Serial(self.port, self.baud)
        # Initiate ReaderThread
        self.reader = ReaderThread(self.serial_port, SerialReaderProtocolRaw)
        # Start reader
        self.reader.start()
        self.recv_queue.queue.clear()
        self.reader.protocol.recv_queue = self.recv_queue

    def close(self):
        self.reader.close()

    def send_cmd(self, data: bytearray, wait_sec: float = 1.):
        self.serial_port.write(data)
        if wait_sec > 0:
            try:
                data = self.reader.protocol.recv_queue.get(block=True, timeout=wait_sec)
                return data
            except Empty:
                print("command no ack")
import struct

class Motor():

    def __init__(self, com:COM, node_id:int):
        self.com = com
        self.node_id = node_id

    @staticmethod
    def floatToBytes(f):
        bs = struct.pack("<f",f)
        return bs

    @staticmethod
    def bytesToFloat(ba:bytearray):
        return struct.unpack("<f",ba)[0]
    
    def calibration(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x02"
        self.com.send_cmd(cmd, wait_sec=0)
 
    def set_current(self, current_A: float):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x03" + self.floatToBytes(current_A)
        self.com.send_cmd(cmd, wait_sec=0)

    def set_velocity(self, velocity: float):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x04" + self.floatToBytes(velocity)
        self.com.send_cmd(cmd, wait_sec=0)

    def set_position(self, angle: float):
        print(angle)
        step=angle/360
        cmd = self.node_id.to_bytes(1, 'little') + b"\x05" + self.floatToBytes(step) + b"\x00"
        self.com.send_cmd(cmd, wait_sec=0)

    def get_FocCurrent(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x21"
        d = self.com.send_cmd(cmd, wait_sec=0.3)
        return self.bytesToFloat(d[:4])
    
    def get_temperature(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x25"
        d = self.com.send_cmd(cmd, wait_sec=0.3)
        return self.bytesToFloat(d[:4])
    def get_position(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x23"
        d = self.com.send_cmd(cmd, wait_sec=0.3)
        return self.bytesToFloat(d[:4])
    
    def get_velocity(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x22"
        d = self.com.send_cmd(cmd, wait_sec=0.3)
        return self.bytesToFloat(d[:4])

    def set_node_id(self, id:int, save:bool):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x11" + id.to_bytes(4, "little") + save.to_bytes(1, "little")
        self.com.send_cmd(cmd, wait_sec=0)

    def enable_temperature(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x7d"
        self.com.send_cmd(cmd, wait_sec=0)


if __name__ == "__main__":

    com = COM('COM3', 115200)
    com.open()
    motor = Motor(com, 0)
  
    print("========run")
    # motor.set_node_id(10, False)
    time.sleep(0.3)
    ret=motor.get_position()
    print(ret)
    # exit()
    try:
        while True:
            motor.set_position(360)
            time.sleep(0.3)
            motor.set_position(0)
            time.sleep(0.3)
    except KeyboardInterrupt as e:
        pass

    com.close()
