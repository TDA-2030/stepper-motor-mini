from serial import Serial, serialutil
from serial.threaded import ReaderThread, Protocol,Packetizer
import time
from queue import Queue, Empty
import struct

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
        self.buffer.clear()

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
        self.reader.protocol.buffer.clear()
        self.serial_port.write(data)
        if wait_sec > 0:
            try:
                data = self.reader.protocol.recv_queue.get(block=True, timeout=wait_sec)
                return data
            except Empty:
                print("command no ack")
        else:
            time.sleep(0.02)


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

    def enable_motor(self, enable:bool):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x01" + enable.to_bytes(1, "little")
        self.com.send_cmd(cmd, wait_sec=0)

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
        """
        angel: 圈数
        """
        step=angle
        cmd = self.node_id.to_bytes(1, 'little') + b"\x05" + self.floatToBytes(step) + b"\x00"
        self.com.send_cmd(cmd, wait_sec=0)

    def set_position_with_time(self, angle: float, _time: float):
        """
        angel: 圈数
        _time: 运行时间
        """
        step=angle
        cmd = self.node_id.to_bytes(1, 'little') + b"\x06" + self.floatToBytes(step) + self.floatToBytes(_time) + b"\x00"
        self.com.send_cmd(cmd, wait_sec=0)

    def set_position_with_velocity(self, angle: float, _vel:float):
        """
        angel: 圈数
        _vel: 每秒转数
        """
        step=angle
        cmd = self.node_id.to_bytes(1, 'little') + b"\x07" + self.floatToBytes(step) + self.floatToBytes(_vel) + b"\x00"
        d = self.com.send_cmd(cmd, wait_sec=0.5)
        if d[0] != self.node_id:
            print(f"id error current id={self.node_id}, recv id={d[0]}")
        return self.bytesToFloat(d[1:5])

    def get_FocCurrent(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x21"
        d = self.com.send_cmd(cmd, wait_sec=0.3)
        if d[0] != self.node_id:
            print(f"id error current id={self.node_id}, recv id={d[0]}")
        return self.bytesToFloat(d[1:5])
    
    def get_temperature(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x25"
        d = self.com.send_cmd(cmd, wait_sec=0.3)
        if d[0] != self.node_id:
            print(f"id error current id={self.node_id}, recv id={d[0]}")
        return self.bytesToFloat(d[1:5])

    def get_position(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x23"
        d = self.com.send_cmd(cmd, wait_sec=0.3)
        if d[0] != self.node_id:
            print(f"id error current id={self.node_id}, recv id={d[0]}")
        return self.bytesToFloat(d[1:5])
    
    def get_velocity(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x22"
        d = self.com.send_cmd(cmd, wait_sec=0.3)
        if d[0] != self.node_id:
            print(f"id error current id={self.node_id}, recv id={d[0]}")
        return self.bytesToFloat(d[1:5])

    def set_node_id(self, id:int, save:bool=False):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x11" + id.to_bytes(4, "little") + save.to_bytes(1, "little")
        self.com.send_cmd(cmd, wait_sec=0)

    def set_rated_current(self, current_A:float, save:bool=False):
        """
        current_A: 电流（安培）
        """
        cmd = self.node_id.to_bytes(1, 'little') + b"\x12" + self.floatToBytes(current_A) + save.to_bytes(1, "little")
        self.com.send_cmd(cmd, wait_sec=0)

    def set_rated_velocity(self, velocity:float, save:bool=False):
        """
        velocity: 每秒转数
        """
        cmd = self.node_id.to_bytes(1, 'little') + b"\x13" + self.floatToBytes(velocity) + save.to_bytes(1, "little")
        self.com.send_cmd(cmd, wait_sec=0)

    def set_rated_acceleration(self, acceleration:float, save:bool=False):
        """
        acceleration: 加速度
        """
        cmd = self.node_id.to_bytes(1, 'little') + b"\x14" + self.floatToBytes(acceleration) + save.to_bytes(1, "little")
        self.com.send_cmd(cmd, wait_sec=0)

    def set_home_position(self):
        """
        设置后会自动保存
        """
        cmd = self.node_id.to_bytes(1, 'little') + b"\x15"
        self.com.send_cmd(cmd, wait_sec=0)

    def set_DEC_Kp(self, Kp:int, save:bool=False):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x17" + Kp.to_bytes(4, 'little') + save.to_bytes(1, 'little')
        self.com.send_cmd(cmd, wait_sec=0)
    def set_DEC_Kv(self, Kv:int, save:bool=False):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x18" + Kv.to_bytes(4, 'little') + save.to_bytes(1, 'little')
        self.com.send_cmd(cmd, wait_sec=0)
    def set_DEC_Ki(self, Ki:int, save:bool=False):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x19" + Ki.to_bytes(4, 'little') + save.to_bytes(1, 'little')
        self.com.send_cmd(cmd, wait_sec=0)
    def set_DEC_Kd(self, Kd:int, save:bool=False):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x1A" + Kd.to_bytes(4, 'little') + save.to_bytes(1, 'little')
        self.com.send_cmd(cmd, wait_sec=0)

    def enable_temperature(self, enable:bool):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x7d" + enable.to_bytes(1, "little")
        self.com.send_cmd(cmd, wait_sec=0)

    def erase_configs(self):
        cmd = self.node_id.to_bytes(1, 'little') + b"\x7e"
        self.com.send_cmd(cmd, wait_sec=0)


class LeadScrew(Motor):

    def __init__(self, com: COM, addr: int, screw_d:float):
        super().__init__(com, addr)
        self.screw_d=screw_d
        self.home_position = 0.0
        print(f"temp={self.get_temperature()}")
        print(f"foc current={self.get_FocCurrent()}")
        print(f"position={self.get_position()}")
        print(f"velocity={self.get_velocity()}")
        self.set_rated_current(1)
        self.set_rated_velocity(40)
        self.set_rated_acceleration(200)

    def set_distance(self, distance: float, speed: int):
        p = distance / self.screw_d
        p += self.home_position
        self.set_position_with_velocity(p, speed / self.screw_d)
    
    def get_distance(self)->float:
        a = self.get_position() - self.home_position
        return (a * self.screw_d)

    def check_home(self, dir:bool):
        self.enable_motor(True)
        time.sleep(0.1)
        self.set_rated_current(0.4)
        self.set_velocity(5.0 if dir else -5.0)
        while abs(self.get_velocity())>1.0:
            time.sleep(0.2)
            print('wait')
        time.sleep(0.5)
        # sym = -1 if dir else 1
        # self.set_position(-0.3)
        # time.sleep(0.5)
        self.home_position = self.get_position()
        print(self.home_position)

        self.set_home_position()
        time.sleep(0.5)

# com = COM('COM3', 115200)
# com.open()
# motor = LeadScrew(com, 0, 6)
# motor.check_home(True)
# r = motor.set_distance(-100, 10)
# time.sleep(10)
# r = motor.get_distance()
# print(r)
# exit()
# try:
#     while True:
#         r = motor.set_position_with_velocity(2, 10)
#         print(r)
#         time.sleep(0.8)
#         r = motor.set_position_with_velocity(0, 10)
#         print(r)
#         time.sleep(0.8)
# except KeyboardInterrupt as e:
#     motor.enable_motor(False)
#     motor.enable_temperature(False)
# exit()

if __name__ == "__main__":

    com = COM('COM3', 115200)
    com.open()
    motor = Motor(com, 0)
  
    print("========run")
    # motor.set_node_id(10, False)
    motor.enable_temperature(True)
    # motor.calibration()
    # exit()
    print(f"temp={motor.get_temperature()}")
    print(f"foc current={motor.get_FocCurrent()}")
    print(f"position={motor.get_position()}")
    print(f"velocity={motor.get_velocity()}")
    motor.set_rated_current(1.5)
    motor.set_rated_acceleration(500)
    motor.set_rated_velocity(40)
    # motor.set_DEC_Kp(250)
    # motor.set_DEC_Kv(80)
    # motor.set_DEC_Ki(100)
    # motor.set_DEC_Kd(500)
    # motor.set_position(0)
    # motor.erase_configs()
    # exit()
    try:
        while True:
            r = motor.set_position(2)
            print(r)
            time.sleep(0.8)
            r = motor.set_position(0)
            print(r)
            time.sleep(0.8)
    except KeyboardInterrupt as e:
        motor.enable_motor(False)
        motor.enable_temperature(False)

    com.close()
