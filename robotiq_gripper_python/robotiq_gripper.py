import serial
from robotiq_gripper_python.gripper_io import GripperIO
from robotiq_gripper_python.modbus_crc import verify_modbus_rtu_crc
import multiprocessing
import time

def gripper_loop(
    comport, baud, control_hz, 
    act_cmd_bytes, act_cmd_bytes_lock, 
    stat_cmd_bytes, stat_cmd_bytes_lock, 
    stat_resp_bytes, stat_resp_bytes_lock, 
    priority_flag,
    stop_event):

    ser = serial.Serial(comport, baud, timeout=0.2)

    try:
        while not stop_event.is_set():

            flag_detected = priority_flag.value == 1

            with act_cmd_bytes_lock:
                act_cmd = act_cmd_bytes[:]
            ser.write(act_cmd)
            rsp = ser.read(8)
            rsp = [int(x) for x in rsp]

            if len(rsp) != 8:
                continue
            if not verify_modbus_rtu_crc(rsp):
                continue
            
            with stat_cmd_bytes_lock:
                stat_cmd = stat_cmd_bytes[:]
            ser.write(stat_cmd)
            rsp = ser.read(21)
            rsp = [int(x) for x in rsp]

            if len(rsp) != 21:
                continue
            if not verify_modbus_rtu_crc(rsp):
                continue
            
            with stat_resp_bytes_lock:
                stat_resp_bytes[:] = rsp

            time.sleep(1.0 / control_hz)

            if flag_detected:
                priority_flag.value = 0

    except KeyboardInterrupt:
        pass

    ser.close()


class RobotiqGripper:
    def __init__(self, num_grippers=1, comport='/dev/ttyUSB0', baud=115200, control_hz=60):

        self._num_grippers = num_grippers
        self._comport = comport
        self._baud = baud
        self._control_hz = control_hz

        try:
            self.ser = serial.Serial(comport, baud, timeout=0.2)
        except:
            self.init_success = False
            return

        self._gripper = []
        self._num_grippers = num_grippers
        for i in range(self._num_grippers):
            self._gripper.append(GripperIO(i))

        self._act_cmd_bytes = multiprocessing.Array('B', self._gripper[0].act_cmd_bytes)
        self._act_cmd_bytes_lock = multiprocessing.Lock()
        self._stat_cmd_bytes = multiprocessing.Array('B', self._gripper[0].stat_cmd_bytes)
        self._stat_cmd_bytes_lock = multiprocessing.Lock()
        self._stat_resp_bytes = multiprocessing.Array('i', [0]*21)
        self._stat_resp_bytes_lock = multiprocessing.Lock()
        self._priority_flag = multiprocessing.Value('i', 0)
        self._stop_event = multiprocessing.Event()

        self._thread = multiprocessing.Process(
            target=gripper_loop, 
            args=(comport, baud, control_hz, 
                    self._act_cmd_bytes, self._act_cmd_bytes_lock, 
                    self._stat_cmd_bytes, self._stat_cmd_bytes_lock, 
                    self._stat_resp_bytes, self._stat_resp_bytes_lock, 
                    self._priority_flag,
                    self._stop_event)
        )
        self._thread.start()

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        self._stop_event.set()
        self._thread.join()

    def start(self, dev=0, timeout=3.0):
        
        self.deactivate_gripper(dev, timeout)
        self.activate_gripper(dev, timeout)



    def print_status(self, dev=0):
        with self._stat_resp_bytes_lock:
            self._gripper[dev].parse_rsp(self._stat_resp_bytes[:])

        print("gACT: ", self._gripper[dev].gACT)
        print("gGTO: ", self._gripper[dev].gGTO)
        print("gSTA: ", self._gripper[dev].gSTA)
        print("gOBJ: ", self._gripper[dev].gOBJ)
        print("gFLT: ", self._gripper[dev].gFLT)
        print("gPR: ", self._gripper[dev].gPR)
        print("gPO: ", self._gripper[dev].gPO)
        print("gCU: ", self._gripper[dev].gCU)
        print()

    def move(self, dev=0, pos=255, vel=255, force=255, block=False):
        assert dev < self._num_grippers, "Invalid device number"

        if block:
            self._priority_flag.value = 1

        self.goto(dev, pos, vel, force)

        if block:
            while self._priority_flag.value == 1 or self.is_moving(dev):

                # lol 
                if self.get_fault_status(dev) != 0:
                    self.activate_gripper(dev)

                time.sleep(1/self._control_hz)
                self.print_status(dev)

    def activate_gripper(self, dev=0, timeout=3.0):
        assert dev < self._num_grippers, "Invalid device number"
        
        self._gripper[dev].activate_gripper()
        with self._act_cmd_bytes_lock:
            self._act_cmd_bytes[:] = self._gripper[dev].act_cmd_bytes
        self._priority_flag.value = 1

        start_time = time.time()
        while self._priority_flag.value == 1 and time.time() - start_time < timeout:
            time.sleep(1/self._control_hz)

    def deactivate_gripper(self, dev=0, timeout=3.0):
        assert dev < self._num_grippers, "Invalid device number"
        
        self._gripper[dev].deactivate_gripper()
        with self._act_cmd_bytes_lock:
            self._act_cmd_bytes[:] = self._gripper[dev].act_cmd_bytes
        self._priority_flag.value = 1

        start_time = time.time()
        while self._priority_flag.value == 1 and time.time() - start_time < timeout:
            time.sleep(1/self._control_hz)

    def activate_emergency_release(self, dev=0, open_gripper=True):
        assert dev < self._num_grippers, "Invalid device number"
        
        self._gripper[dev].activate_emergency_release(open_gripper)
        with self._act_cmd_bytes_lock:
            self._act_cmd_bytes[:] = self._gripper[dev].act_cmd_bytes

    def deactivate_emergency_release(self, dev=0):
        assert dev < self._num_grippers, "Invalid device number"
        
        self._gripper[dev].deactivate_emergency_release()
        with self._act_cmd_bytes_lock:
            self._act_cmd_bytes[:] = self._gripper[dev].act_cmd_bytes

    def goto(self, dev=0, pos=0.0, vel=1.0, force=1.0):
        assert dev < self._num_grippers, "Invalid device number"

        self._gripper[dev].goto(pos, vel, force)
        with self._act_cmd_bytes_lock:
            self._act_cmd_bytes[:] = self._gripper[dev].act_cmd_bytes

    def stop(self, dev=0):
        assert dev < self._num_grippers, "Invalid device number"
        
        self._gripper[dev].stop()
        with self._act_cmd_bytes_lock:
            self._act_cmd_bytes[:] = self._gripper[dev].act_cmd_bytes

    def is_ready(self, dev=0):
        assert dev < self._num_grippers, "Invalid device number"
        
        with self._stat_resp_bytes_lock:
            self._gripper[dev].parse_rsp(self._stat_resp_bytes[:])

        return self._gripper[dev].is_ready()

    def is_reset(self, dev=0):
        assert dev < self._num_grippers, "Invalid device number"
        
        with self._stat_resp_bytes_lock:
            self._gripper[dev].parse_rsp(self._stat_resp_bytes[:])

        return self._gripper[dev].is_reset()

    def is_moving(self, dev=0):
        assert dev < self._num_grippers, "Invalid device number"
        
        with self._stat_resp_bytes_lock:
            self._gripper[dev].parse_rsp(self._stat_resp_bytes[:])

        return self._gripper[dev].is_moving()

    def is_stopped(self, dev=0):
        return not self.is_moving(dev)

    def object_detected(self, dev=0):
        assert dev < self._num_grippers, "Invalid device number"
        
        with self._stat_resp_bytes_lock:
            self._gripper[dev].parse_rsp(self._stat_resp_bytes[:])

        return self._gripper[dev].object_detected()

    def get_fault_status(self, dev=0):
        assert dev < self._num_grippers, "Invalid device number"
        
        with self._stat_resp_bytes_lock:
            self._gripper[dev].parse_rsp(self._stat_resp_bytes[:])

        return self._gripper[dev].get_fault_status()

    def get_pos(self, dev=0):
        assert dev < self._num_grippers, "Invalid device number"
        
        with self._stat_resp_bytes_lock:
            self._gripper[dev].parse_rsp(self._stat_resp_bytes[:])

        return self._gripper[dev].get_pos()

    def get_req_pos(self, dev=0):
        assert dev < self._num_grippers, "Invalid device number"
        
        with self._stat_resp_bytes_lock:
            self._gripper[dev].parse_rsp(self._stat_resp_bytes[:])

        return self._gripper[dev].get_req_pos()

    def get_current(self, dev=0):
        assert dev < self._num_grippers, "Invalid device number"
        
        with self._stat_resp_bytes_lock:
            self._gripper[dev].parse_rsp(self._stat_resp_bytes[:])

        return self._gripper[dev].get_current()
