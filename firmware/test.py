
import time
import struct

from threading import Lock
import time

import minimalmodbus
import serial
import serial.tools.list_ports
import traceback

def get_serial_port():
    port = serial.tools.list_ports.comports()
    return [p.__dict__ for p in port]


class RS485():
    def __init__(self, serial, slave_addr=6) -> None:
        self.ser = serial
        self.slave_addr = slave_addr
        self.is_connected:bool = False
        self.lock = Lock()

    def connect(self):
        try:
            print(f"Connecting RS485 addr:{self.slave_addr}")
            # self.master = modbus_tcp.TcpMaster(ip, port)
            # self.master.set_timeout(5.0)
            self.master = minimalmodbus.Instrument(self.ser, self.slave_addr)
            
            self.is_connected = True
            return True
        except:
            traceback.print_exc()
            return False

    def disconnect(self):
        self.ser.close()
        del self.master
        del self.ser
        self.is_connected = False

    def read_register(self, addr:int):
        ret = self.master.read_register(addr)
        # return self.master.execute(self.slave_addr, cst.READ_HOLDING_REGISTERS, addr, 1)
        return ret
        
    def write_register(self, addr:int, value:int):
        ret = self.master.write_register(addr, value, functioncode=6)
        # return self.master.execute(self.slave_addr, cst.WRITE_SINGLE_REGISTER, addr, output_value=value)
        return ret

    def write_long(self, addr:int, value:int, number_of_bytes:int=2):
        ret = self.master.write_long(addr, value, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP, number_of_registers=number_of_bytes)
        # return self.master.execute(self.slave_addr, cst.WRITE_SINGLE_REGISTER, addr, output_value=value)
        return ret
    def write_float(self, addr:int, value:float):
        ret = self.master.write_float(addr, value, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP)
        # return self.master.execute(self.slave_addr, cst.WRITE_SINGLE_REGISTER, addr, output_value=value)
        return ret
    
    def read_float(self, addr:int):
        ret = self.master.read_float(addr, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP)
        # return self.master.execute(self.slave_addr, cst.WRITE_SINGLE_REGISTER, addr, output_value=value)
        return ret
    
    def read_long(self, addr:int, number_of_bytes:int=2):
        ret = self.master.read_long(addr, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP, number_of_registers=number_of_bytes)
        return ret


class Motor():

    def __init__(self, com:RS485):
        self.com = com
        self.base_addr = 0x1000

    @staticmethod
    def floatToBytes(f):
        bs = struct.pack("<f",f)
        return bs

    @staticmethod
    def bytesToFloat(ba:bytearray):
        return struct.unpack("<f",ba)[0]

    def enable_motor(self, enable:bool):
        self.com.write_register(self.base_addr+1, int(enable))

    def calibration(self):
        self.com.write_register(self.base_addr+2, 1)
 
    def set_current(self, current_A: float):
        self.com.write_float(self.base_addr+3, current_A)

    def set_velocity(self, velocity: float):
        self.com.write_float(self.base_addr+5, velocity)

    def set_position(self, angle: float):
        """
        angel: 圈数
        """
        self.com.write_float(self.base_addr+7, angle)

    def set_position_with_time(self, angle: float, _time: float):
        """
        angel: 圈数
        _time: 运行时间
        """
        self.com.write_long(self.base_addr+9, int.from_bytes(self.floatToBytes(angle) + self.floatToBytes(_time), 'little'), 4)

    def set_position_with_velocity(self, angle: float, _vel:float):
        """
        angel: 圈数
        _vel: 每秒转数
        """
        # self.com.write_float(self.base_addr+13, angle)
        # time.sleep(0.1)
        # self.com.write_float(self.base_addr+15, _vel)

        self.com.write_long(self.base_addr+13, 
                            struct.unpack("<Q",self.floatToBytes(angle) + self.floatToBytes(_vel))[0], 4)


    def get_FocCurrent(self):
        return self.com.read_float(self.base_addr+3)
    
    def get_temperature(self):
        return self.com.read_float(self.base_addr+35)
    
    def get_voltage(self):
        return self.com.read_float(self.base_addr+37)

    def get_position(self):
        return self.com.read_float(self.base_addr+7)

    def get_velocity(self):
        return self.com.read_float(self.base_addr+5)

    def set_node_id(self, id:int):
        return self.com.write_register(self.base_addr+17, id)

    def get_node_id(self):
        return self.com.read_register(self.base_addr+17)

    def get_HomeOffset(self):
        return self.com.read_float(self.base_addr+24)
    
    def get_reverse_direction(self):
        return self.com.read_register(self.base_addr+39)

    def set_rated_current(self, current_A:float):
        """
        current_A: 电流（安培）
        """
        self.com.write_float(self.base_addr+18, current_A)

    def set_rated_velocity(self, velocity:float):
        """
        velocity: 每秒转数
        """
        self.com.write_float(self.base_addr+20, velocity)

    def set_rated_acceleration(self, acceleration:float):
        """
        acceleration: 加速度
        """
        self.com.write_float(self.base_addr+22, acceleration)

    def set_home_position(self):
        """
        设置后会自动保存
        """
        return self.com.write_register(self.base_addr+24, 1)

    def set_DEC_Kp(self, Kp:int):
        self.com.write_long(self.base_addr+26, Kp)
    def set_DEC_Kv(self, Kv:int):
        self.com.write_long(self.base_addr+28, Kv)
    def set_DEC_Ki(self, Ki:int):
        self.com.write_long(self.base_addr+30, Ki)
    def set_DEC_Kd(self, Kd:int):
        self.com.write_long(self.base_addr+32, Kd)

    def get_DEC_Kp(self):
        return self.com.read_long(self.base_addr+26)
    def get_DEC_Kv(self):
        return self.com.read_long(self.base_addr+28)
    def get_DEC_Ki(self):
        return self.com.read_long(self.base_addr+30)
    def get_DEC_Kd(self):
        return self.com.read_long(self.base_addr+32)

    def enable_temperature(self, enable:bool):
        return self.com.write_register(self.base_addr+35, int(enable))
    
    def reverse_direction(self, enable:bool):
        return self.com.write_register(self.base_addr+39, int(enable))

    def save_config(self):
        return self.com.write_register(self.base_addr+125, 1)

    def erase_configs(self):
        return self.com.write_register(self.base_addr+126, 1)
    
    def reboot(self):
        return self.com.write_register(self.base_addr+127, 1)
    
    def show_info(self):
        print("-----motor info-----")
        print(f"reverse_direction={self.get_reverse_direction()}")
        print(f"temp={self.get_temperature()}")
        print(f"voltage={self.get_voltage()}")
        print(f"foc current={self.get_FocCurrent()}")
        print(f"position={self.get_position()}")
        print(f"velocity={self.get_velocity()}")
        print(f"homeoffset={self.get_HomeOffset()}")
        print(f"DEC_Kp={self.get_DEC_Kp()}")
        print(f"DEC_Kv={self.get_DEC_Kv()}")
        print(f"DEC_Ki={self.get_DEC_Ki()}")
        print(f"DEC_Kd={self.get_DEC_Kd()}")
        print("-----end-----")


class LeadScrew(Motor):

    def __init__(self, com: RS485, screw_d:float):
        """
        screw_d: 丝杆导程 (mm)
        """
        super().__init__(com)
        self.screw_d=screw_d
        self.show_info()
        self.set_rated_current(0.5)
        self.set_rated_velocity(40)
        self.set_rated_acceleration(200)

    def set_distance(self, distance: float, speed: int):
        """
        distance: 绝对距离(mm)
        speed: 速度(mm/s)
        """
        p = distance / self.screw_d
        self.set_position_with_velocity(p, speed / self.screw_d)
    
    def get_distance(self)->float:
        """
        返回: 绝对距离(mm)
        """
        a = self.get_position()
        return (a * self.screw_d)

    def check_home(self, dir:bool):
        """
        检查原点
        dir: 回原点方向
        """
        self.enable_motor(True)
        time.sleep(0.1)
        self.set_rated_current(0.4)
        self.set_velocity(3.0 if dir else -3.0)
        while abs(self.get_velocity())>1.0:
            time.sleep(0.2)
            print('wait')
        time.sleep(0.5)
        self.set_home_position()
        time.sleep(0.5)


def test_leadscrew(com):
    motor = LeadScrew(com, 12*42.4/40)
    motor.reverse_direction(False)

    motor.check_home(False)

    try:
        while True:
            r = motor.set_distance(2, 10)
            print(motor.get_distance())
            time.sleep(0.8)
            r = motor.set_distance(0, 10)
            print(motor.get_distance())
            time.sleep(0.8)
    except KeyboardInterrupt as e:
        motor.enable_motor(False)
        motor.enable_temperature(False)

def rw_test():
    d = RS485(1)
    d.connect('COM3', 115200)
    lt = time.perf_counter()
    import random
    while True:
        vv = random.randint(0, 65535)
        d.write_register(0x1000, vv)
        v = d.read_register(0x1000 + 0)
        t = time.perf_counter()
        print(v, t-lt)
        if v != vv:
            break
        lt = t
        time.sleep(0.03)

if __name__ == "__main__":

    for c in get_serial_port():
        print(c)
    ser = serial.Serial('COM3', 115200, timeout=0.5)

    d = RS485(ser, 1)
    d.connect()
    test_leadscrew(d)
    exit()
    motor = Motor(d)

    # d42 = RS485(ser, 1)
    # d42.connect()
    # motor42 = Motor(d42)

    print("========run")
    # motor.set_node_id(5)
    print(f"node_id={motor.get_node_id()}")
    # print(f"42 node_id={motor42.get_node_id()}")
    # motor.save_config()
    # motor.reboot()
    # exit()
    motor.enable_temperature(True)
    motor.reverse_direction(False)
    # motor.calibration()
    # exit()
    motor.show_info()


    motor.set_rated_current(0.5)
    # motor42.set_rated_acceleration(500)
    # motor42.set_rated_velocity(50)
    motor.set_rated_acceleration(100)
    motor.set_rated_velocity(3)
    # motor.set_DEC_Kp(300)
    # # motor.set_DEC_Kv(80)
    # motor.set_DEC_Ki(100)
    # motor.set_DEC_Kd(1800)
    # motor.set_position(0)
    # # motor.erase_configs()
    # motor.save_config()
    # exit()
    motor.set_home_position()
    time.sleep(0.1)
    r = motor.get_position()
    print(r)
    try:
        while True:
            # r = motor42.set_position_with_velocity(2, 2)
            time.sleep(0.1)
            r = motor.set_position_with_time(1, 2)
            time.sleep(2)
            r = motor.get_position()
            print(r)

            # r = motor42.set_position_with_velocity(0, 2)
            time.sleep(0.1)
            r = motor.set_position_with_time(2, 1)
            time.sleep(1)
            r = motor.get_position()
            print(r)
    except KeyboardInterrupt as e:
        motor.enable_motor(False)
        # motor42.enable_motor(False)
        motor.enable_temperature(False)

    d.disconnect()
