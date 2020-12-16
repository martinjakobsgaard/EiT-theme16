import serial
import rtde_receive
import rtde_control
import threading
import time
import struct

ip = "192.168.0.212"

FRONT = 49
BACK = 50
STOP = 51

# Far away
p1 = [0.0179293, -0.488497, 0.4558, -2.57862, 1.79077, 0.0727724]
# Approach
p2 = [-0.123498, -0.503326, 0.285541, 2.56927, -1.68485, 0.0124782]
# First contact
p3 = [0.117794, -0.426809, 0.256406, 2.56927, -1.68492, 0.0123743]
# Lift slightly up
p4 = [0.117805, -0.426817, 0.277613, -2.4653, 1.64754, -0.0285471]
# Forklift sliding
p5 = [0.36203, -0.33961, 0.302266, -2.51342, 1.68852, 0.0225389]
# Forklift done
p6 = [0.42865, -0.31833, 0.310124, -2.53676, 1.70214, -0.000315614]
# Lifted up material
p7 = [0.428629, -0.318319, 0.331533, -2.41957, 1.62123, -0.157895]
# Moved material away
p8 = [0.439175, -0.431889, 0.505297, -2.41965, 1.62118, -0.157964]


# Place points
p9 = [0.439181, -0.431892, 0.505278, -1.70209, 2.41975, -0.269813]
p10 = [0.439144, -0.556322, 0.505254, -0.702129, 2.94703, -0.352995]
p11 = [-0.0042988, -0.598356, 0.505273, -0.702218, 2.947, -0.353108]

p12 = [-0.288039, -0.668178, 0.505228, -0.702151, 2.94693, -0.353011]
p13 = [-0.369101, -0.43335, 0.505187, -0.702024, 2.94685, -0.352988]
p14 = [-0.309318, -0.403673, 0.278585, -0.702832, 2.95168, -0.33494]
p15 = [-0.30932, -0.403689, 0.223064, -0.7126, 3.01997, -0.00527375]
p16 = [-0.477062, -0.0661112, 0.223068, -0.712635, 3.01995, -0.00528498]

p17 = [-0.483899, -0.052367, 0.473396, -0.71254, 3.0199, -0.00532701]
p18 = [-0.176069, -0.671882, 0.473417, -0.712558, 3.01996, -0.0052031]


class gripper_control:
    writer = serial.Serial('/dev/ttyUSB0', 9600)
    rec = rtde_receive.RTDEReceiveInterface(ip)
    control = rtde_control.RTDEControlInterface(ip)
    move_back = False
    move_forward = False

    def __init__(self):
        time.sleep(1)

    def robot_movement(self):
        time.sleep(5)
        self.control.moveL(p1, 0.4, 0.4)
        self.control.moveL(p2, 0.4, 0.4)
        self.control.moveL(p3, 0.4, 0.4)
        self.control.moveL(p4, 0.4, 0.4)
        self.move_back = True
        #self.control.moveL(p5, 2.5, 0.01)
        self.control.moveL(p6, 2.5, 0.01)
        self.control.moveL(p7, 0.4, 0.4)
        self.control.moveL(p8, 0.4, 0.4)

        #Place part
        self.control.moveL(p9, 0.4, 0.4)
        self.control.moveL(p10, 0.4, 0.4)
        self.control.moveL(p11, 0.4, 0.4)
        self.control.moveL(p12, 0.4, 0.4)
        self.control.moveL(p13, 0.4, 0.4)
        self.control.moveL(p14, 0.4, 0.4)
        self.control.moveL(p15, 0.4, 0.4)
        # Move back place back
        self.move_forward = True
        self.control.moveL(p16, 2.5, 0.01)
        self.control.moveL(p17, 0.4, 0.4)
        self.control.moveL(p18, 0.4, 0.4)

    def gripper_movement(self):
        self.writer.write(struct.pack('>BB', FRONT, 255))
        time.sleep(10)
        while True:
            if self.move_back:
                time.sleep(1)
                self.writer.write(struct.pack('>BB', BACK, 155))
                self.move_back = False
                time.sleep(1.9)
                self.writer.write(struct.pack('>BB', BACK, 255))
                time.sleep(4)
                self.writer.write(struct.pack('>BB', BACK, 155))
                time.sleep(1.9)
            if self.move_forward:
                time.sleep(1)
                self.writer.write(struct.pack('>BB', FRONT, 155))
                self.move_forward = False
                time.sleep(1.9)
                self.writer.write(struct.pack('>BB', FRONT, 255))
                time.sleep(4)
                self.writer.write(struct.pack('>BB', FRONT, 155))
                time.sleep(1.9)
                break
        #print("Done with while true")

    def main(self):
        t1 = threading.Thread(target=self.robot_movement)
        t2 = threading.Thread(target=self.gripper_movement)
        t1.start()
        t2.start()
        t1.join()
        t2.join()


if __name__ == "__main__":
    robot = gripper_control()
    robot.main()
