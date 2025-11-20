#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import serial.tools.list_ports
import logging

logger = logging.getLogger(__name__)

class Dynamixel:
    ver = '23.10.27'

    ## Control table address
    # 0	2	Model Number	R	1,020	-	-
    # 2	4	Model Information	R	-	-	-
    ADDR_FIRMWARE_VERSION = 6
    ADDR_RETURN_DELAY_TIME  = 9 # 1 RW	250	0 ~ 254	2 [μsec]
    ADDR_DRIVE_MODE = 10    # 1 RW	0	0 ~ 5	-
    #     1: Reverse mode
    ADDR_OPERATING_MODE     = 11    # 1 RW	3	0 ~ 16	-
    #     0: Current Control Mode
    #     1: Velocity Control Mode
    #     3: Position Control Mode
    #     16: PWM Control Mode (Voltage Control Mode)
    # 12	1	Secondary(Shadow) ID	RW	255	0 ~ 252	-
    # 13	1	Protocol Type	RW	2	1 ~ 2	-
    # 20	4	Homing Offset	RW	0	-1,044,479 ~
    # 1,044,479	1 [pulse]
    # 24	4	Moving Threshold	RW	10	0 ~ 1,023	0.229 [rev/min]
    # 31	1	Temperature Limit	RW	80	0 ~ 100	1 [°C]
    # 32	2	Max Voltage Limit	RW	160	95 ~ 160	0.1 [V]
    # 34	2	Min Voltage Limit	RW	95	95 ~ 160	0.1 [V]
    # 36	2	PWM Limit	RW	885	0 ~ 885	0.113 [%]
    # 38	2	Current Limit	RW	1,193	0 ~ 1,193	2.69 [mA]
    # 44	4	Velocity Limit	RW	200	0 ~ 1,023	0.229 [rev/min]
    ADDR_MAX_POSITION_LIMIT = 48    # 4	RW	4,095	0 ~ 4,095	1 [pulse]
    ADDR_MIN_POSITION_LIMIT = 52    # 4	RW	0	0 ~ 4,095	1 [pulse]
    # 60	1	Startup Configuration	RW	0	3	-
    # 63	1	Shutdown	RW	52	-	-
    ADDR_TORQUE_ENABLE      = 64    # 1 RW	0	0 ~ 1	-
    ADDR_LED = 65   # 1 RW	0	0 ~ 1	-
    ADDR_STATUS_RETURN_LEVEL = 68   # 1 RW	2	0 ~ 2	-
    #     0: PING Instruction
    #     1: PING Instruction, READ Instruction
    #     2: All Instructions	
    # 69	1	Registered Instruction	R	0	0 ~ 1	-
    # 70	1	Hardware Error Status	R	0	-	-
    # 76	2	Velocity I Gain	RW	1,920	0 ~ 16,383	-
    # 78	2	Velocity P Gain	RW	100	0 ~ 16,383	-
    # 80	2	Position D Gain	RW	0	0 ~ 16,383	-
    # 82	2	Position I Gain	RW	0	0 ~ 16,383	-
    # 84	2	Position P Gain	RW	800	0 ~ 16,383	-
    # 88	2	Feedforward 2nd Gain	RW	0	0 ~ 16,383	-
    # 90	2	Feedforward 1st Gain	RW	0	0 ~ 16,383	-
    # 98	1	Bus Watchdog	RW	0	1 ~ 127	20 [msec]
    ADDR_GOAL_PWM           = 100   # 0 RW	-	-PWM Limit(36) ~ PWM Limit(36)	-
    ADDR_GOAL_CURRENT       = 102   # 2 RW	-	-Current Limit(38) ~ Current Limit(38)	2.69 [mA]
    ADDR_GOAL_VELOCITY      = 104  # 4 RW	-	-Velocity Limit(44) ~ Velocity Limit(44)	0.229 [rev/min]
    # 108	4	Profile Acceleration	RW	0	0 ~ 32,767 0 ~ 32,737	214.577 [rev/min2] 1 [ms]
    # 112	4	Profile Velocity	RW	0	0 ~ 32,767	0.229 [rev/min]
    ADDR_GOAL_POSITION      = 116   # 4 RW	-	Min Position Limit(52) ~ Max Position Limit(48)	1 [pulse]
    # 120	2	Realtime Tick	R	-	0 ~ 32,767	1 [msec]
    ADDR_PRESENT_PWM     = 124      # 2 R	-	-	-
    ADDR_PRESENT_CURRENT = 126      # 2 R	-	-	2.69 [mA]
    ADDR_PRESENT_VELOCITY = 128     # 4	R	-	-	0.229 [rev/min]
    ADDR_PRESENT_POSITION   = 132   # 4	R	-	-	1 [pulse]
    # 136	4	Velocity Trajectory	R	-	-	0.229 [rev/min]
    # 140	4	Position Trajectory	R	-	-	1 [pulse]
    # 144	2	Present Input Voltage	R	-	-	0.1 [V]
    # 146	1	Present Temperature	R	-	-	1 [°C]
    # 147	1	Backup Ready	R	-	0 ~ 1	-
    
    # Protocol version
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
    # COMM_SUCCESS                = 0;            % Communication Success result value
    # COMM_TX_FAIL                = -1001;        % Communication Tx Failed
    
    def __init__(self, device_name, baud=57600):
        self.DEVICENAME = device_name
        self.BAUDRATE = baud

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if not self.portHandler.openPort():
            logging.debug("Failed to open the port")
            self.portHandler.closePort()
            quit()

        # Set port baudrate
        if  not self.portHandler.setBaudRate(self.BAUDRATE):
            logging.debug("Failed to change the baudrate")
            self.portHandler.closePort()
            quit()

        self.fastMode = True

    def __del__(self):
        # Close port
        if hasattr(self, 'portHandler'):
            self.portHandler.closePort()

    def reboot(self, DXL_IDs):
        if not hasattr(DXL_IDs, "__iter__"):
            DXL_IDs = [DXL_IDs]
        for i in range(len(DXL_IDs)):
            dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, DXL_IDs[i])
            if dxl_comm_result != COMM_SUCCESS:
                logging.debug("%s", self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                logging.debug("%s", self.packetHandler.getRxPacketError(dxl_error))
            time.sleep(0.1)

    def setRecommendedValue(self, DXL_IDs):
        if not hasattr(DXL_IDs, "__iter__"):
            DXL_IDs = [DXL_IDs]
        self.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))
        self.writeReturnDelayTime(DXL_IDs, [0] * len(DXL_IDs))
        self.writeStatusReturnLevel(DXL_IDs, [1] * len(DXL_IDs))
        # self.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))


    def readFirmwareVersion(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_FIRMWARE_VERSION, 1, DXL_IDs)
    def readReturnDelayTime(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_RETURN_DELAY_TIME, 1, DXL_IDs)
    def readDriveMode(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_DRIVE_MODE, 1, DXL_IDs)
    def readOperatingMode(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_OPERATING_MODE, 1, DXL_IDs)
    def readMaxPositionLimit(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_MAX_POSITION_LIMIT, 4, DXL_IDs)
    def readMinPositionLimit(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_MIN_POSITION_LIMIT, 4, DXL_IDs)
    def readTorqueEnable(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_TORQUE_ENABLE, 1, DXL_IDs)
    def readLED(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_LED, 1, DXL_IDs)
    def readStatusReturnLevel(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_STATUS_RETURN_LEVEL, 1, DXL_IDs)

    def writeReturnDelayTime(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_RETURN_DELAY_TIME, 1, DXL_IDs, data)
    def writeDriveMode(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_DRIVE_MODE, 1, DXL_IDs, data)
    def writeOperatingMode(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_OPERATING_MODE, 1, DXL_IDs, data)
    def writeMaxPositionLimit(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_MAX_POSITION_LIMIT, 4, DXL_IDs, data)
    def writeMinPositionLimit(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_MIN_POSITION_LIMIT, 4, DXL_IDs, data)
    def writeTorqueEnable(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_TORQUE_ENABLE, 1, DXL_IDs, data)
    def writeLED(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_LED, 1, DXL_IDs, data)
    def writeStatusReturnLevel(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_STATUS_RETURN_LEVEL, 1, DXL_IDs, data)

    def readPresentPWM(self, DXL_IDs, data):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_PWM, 2, DXL_IDs)
    def readPresentVelocity(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_VELOCITY, 4, DXL_IDs)
    def readPresentPosition(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_POSITION, 4, DXL_IDs)
    def readPresentCurrent(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_CURRENT, 2, DXL_IDs)

    def writeGoalPWM(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_PWM, 2, DXL_IDs, data)
    def writeGoalCurrent(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_CURRENT, 2, DXL_IDs, data)
    def writeGoalVelocity(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_VELOCITY, 4, DXL_IDs, data)
    def writeGoalPosition(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_POSITION, 4, DXL_IDs, data)

    def groupAsyncWrite(self, ADDR, byte, DXL_IDs, data):
        if not hasattr(DXL_IDs, "__iter__"):
            DXL_IDs = [DXL_IDs]
            data = [data]
        for i, DXL_ID in enumerate(DXL_IDs):
            if byte == 1:
                dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, DXL_ID, ADDR, int(data[i]))
            elif byte == 2:
                dxl_comm_result = self.packetHandler.write2ByteTxOnly(self.portHandler, DXL_ID, ADDR, int(data[i]))
            elif byte == 4:
                dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, DXL_ID, ADDR, int(data[i]))
            if dxl_comm_result != COMM_SUCCESS:
                logging.debug("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            time.sleep(0.05)
    def groupAsyncRead(self, ADDR, byte, DXL_IDs):
        data = [0]*len(DXL_IDs)
        for i, DXL_ID in enumerate(DXL_IDs):
            if byte == 1:
                data[i], dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, DXL_ID, ADDR)
            elif byte == 2:
                data[i], dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, ADDR)
            elif byte == 4:
                data[i], dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR)
            if dxl_comm_result != COMM_SUCCESS:
                logging.debug("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                logging.debug("%s" % packetHandler.getRxPacketError(dxl_error))
        return data
    def groupSyncWrite(self, ADDR, byte, DXL_IDs, data):
        if not hasattr(DXL_IDs, "__iter__"):
            DXL_IDs = [DXL_IDs]
            data = [data]
        groupSyncWritePacket = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR, byte)
        # self.groupSyncWriteGoalPosition.clearParam()
        for DXL_ID in DXL_IDs:
            iData = int(data[DXL_IDs.index(DXL_ID)])
            match byte:
                case 4:
                    param = [ iData&0xFF, (iData>>8)&0xFF, (iData>>16)&0xFF, (iData>>24)&0xFF ]
                case 2:
                    param = [ iData&0xFF, (iData>>8)&0xFF ]
                case _:
                    param = [ iData&0xFF ]
            # dxl_addparam_result = groupSyncWritePacket.addParam(DXL_ID, self.signed_hex2int(param, 8))
            dxl_addparam_result = groupSyncWritePacket.addParam(DXL_ID, param)
            if dxl_addparam_result != True:
                logging.debug("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID)
        dxl_comm_result = groupSyncWritePacket.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            logging.debug("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
    
    def groupSyncRead(self, ADDR, byte, DXL_IDs):
        if not hasattr(DXL_IDs, "__iter__"):
            DXL_IDs = [DXL_IDs]
        data = [0]*len(DXL_IDs)
        groupSyncReadPacket = GroupSyncRead(self.portHandler, self.packetHandler,ADDR, byte)
        # groupSyncReadPacket.clearParam()
        for DXL_ID in DXL_IDs:
            dxl_addparam_result = groupSyncReadPacket.addParam(DXL_ID)
            if dxl_addparam_result != True:
                logging.debug("[ID:%03d] groupSyncRead addparam failed" % DXL_ID)
        if self.fastMode:
            dxl_comm_result = groupSyncReadPacket.fastSyncRead()
        else:
            dxl_comm_result = groupSyncReadPacket.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            logging.debug("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        for num,DXL_ID in enumerate(DXL_IDs):
            dxl_getdata_result = groupSyncReadPacket.isAvailable(DXL_ID, ADDR, byte)
            if dxl_getdata_result != True:
                logging.debug("[ID:%03d] groupSyncRead getdata failed" % DXL_ID)
            data[num] = groupSyncReadPacket.getData(DXL_ID, ADDR, byte)
            match byte:
                case 4:
                    data[num] = self.signed_hex2int(data[num], 32)
                case 2:
                    data[num] = self.signed_hex2int(data[num], 16)
                case _:
                    data[num] = data[num]&0xFF
        return data

    def groupBulkRead(self, req_list):# [addr, byte, id]
        # ID, ADDR, LEN
        if not hasattr(req_list, "__iter__"):
            return None
        data = [0]*len(req_list)
        # logging.info(data)
        groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)
        # groupSyncReadPacket.clearParam()
        for req in req_list:
            groupBulkRead.addParam(req[2], req[0], req[1])
        if self.fastMode:
            dxl_comm_result = groupBulkRead.fastBulkRead()
        else:
            dxl_comm_result = groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            logging.debug("%s", self.packetHandler.getTxRxResult(dxl_comm_result))
        for num, req in enumerate(req_list):
            dxl_getdata_result = groupBulkRead.isAvailable(req[2], req[0], req[1])
            # logging.info(dxl_getdata_result)
            if not dxl_getdata_result:
                logging.debug("[ID:%03d, ADDR:%d] groupBulkRead getdata failed", req[2], req[0])
            match  req[1]:
                case 4:
                    data[num] = self.signed_hex2int(groupBulkRead.getData(req[2], req[0], req[1]), 32)
                case 2:
                    data[num] = self.signed_hex2int(groupBulkRead.getData(req[2], req[0], req[1]), 16)
                case _:
                    data[num] = groupBulkRead.getData(req[2], req[0], req[1])&0xFF
                    # logging.info("%d", req[1])
        return data
    
    @classmethod
    def open_ports_by_serial_number(cls, serial_number, baud=57600):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return cls(port.device, baud)
    @staticmethod
    def device_name_from_serial_number(serial_number):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return port.device
    @staticmethod
    def show_list_ports():
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid is None:
                port.serial_number = "None"
                port.vid = 0
                port.pid = 0
            logging.info('Name:%s, Serial Number:%s, VID:PID:%04X:%04X, Manufacturer:%s'%(
                port.device,
                port.serial_number,
                port.vid,
                port.pid,
                port.manufacturer) )

    @staticmethod
    def signed_hex2int( signed_hex, digit ):
        signed = 0x01 << (digit-1)
        mask = 0x00
        for num in range(digit):
            mask = mask | (0x01<<num)
        signed_int = (int(signed_hex^mask)*-1)-1  if (signed_hex & signed) else int(signed_hex)
        return signed_int

def testPositionControl(dyn, DXL_IDs):
    dyn.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))
    dyn.writeOperatingMode(DXL_IDs, [3] * len(DXL_IDs))
    dyn.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))

    logging.info( dyn.readPresentPosition(DXL_IDs) )
    dyn.writeGoalPosition(DXL_IDs, dyn.readMaxPositionLimit(DXL_IDs))
    time.sleep(1.0)
    logging.info( dyn.readPresentPosition(DXL_IDs) )
    dyn.writeGoalPosition(DXL_IDs, dyn.readMinPositionLimit(DXL_IDs))
    time.sleep(1.0)
    logging.info( dyn.readPresentPosition(DXL_IDs) )
    dyn.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))

    num = 100
    for i in range(3):
        start = time.time()
        for i in range(num):
            dyn.readPresentPosition(DXL_IDs)
            dyn.readPresentVelocity(DXL_IDs)
        logging.info("elapsed_time(%s read):%s[msec]", num, time.time()*1000 - start*1000)
        logging.info( dyn.readPresentPosition(DXL_IDs) )
        logging.info( dyn.readPresentVelocity(DXL_IDs) )

def testVelocityControl(dyn, DXL_IDs):
    dyn.writeTorqueEnable(DXL_IDs, [0] * 2)
    dyn.writeOperatingMode(DXL_IDs, [1] * 2)
    dyn.writeTorqueEnable(DXL_IDs, [1] * 2)

    dyn.writeGoalVelocity(DXL_IDs, [10] * 2)
    time.sleep(1.0)
    logging.info(dyn.readPresentVelocity(DXL_IDs))
    dyn.writeGoalVelocity(DXL_IDs, [-10] * 2)
    time.sleep(1.0)
    logging.info(dyn.readPresentVelocity(DXL_IDs))
    dyn.writeGoalVelocity(DXL_IDs, [0,0])
    time.sleep(.5)
    
    dyn.writeTorqueEnable(DXL_IDs, [0, 0])

def testReadWriteCurrent(dyn, DXL_IDs):
    dyn.writeTorqueEnable(DXL_IDs, [0, 0])
    dyn.writeOperatingMode(DXL_IDs, [0] * 2)
    dyn.writeTorqueEnable(DXL_IDs, [1, 1])

    dyn.writeGoalCurrent(DXL_IDs, [-5] * 2)
    time.sleep(1.0)
    logging.info(dyn.readPresentCurrent(DXL_IDs))
    dyn.writeGoalCurrent(DXL_IDs, [3] * 2)
    time.sleep(1.0)
    logging.info(dyn.readPresentCurrent(DXL_IDs))
    dyn.writeGoalCurrent(DXL_IDs, [0] * 2)
    time.sleep(0.5)
    
    dyn.writeTorqueEnable(DXL_IDs, [0, 0])

def testReadSettings(dyn, DXL_IDs):
    # To update the firmware, use the Dynamixel Wizard 2.0 software provided by ROBOTIS.
    logging.info("FirmwareVersion: %s", dyn.readFirmwareVersion(DXL_IDs))
    logging.info("ReturnDelayTime: %s", dyn.readReturnDelayTime(DXL_IDs))
    logging.info("DriveMode: %s", dyn.readDriveMode(DXL_IDs))
    logging.info("OperatingMode: %s", dyn.readOperatingMode(DXL_IDs))
    logging.info("TorqueEnable: %s", dyn.readTorqueEnable(DXL_IDs))
    logging.info("LED: %s", dyn.readLED(DXL_IDs))
    logging.info("StatusReturnLevel: %s", dyn.readStatusReturnLevel(DXL_IDs))

def testGroupBulkRead(dyn, DXL_IDs):
    # GroupBulkRead does not work well with many actuators.

    # [Addr, Byte, ID]
    req_list = [
        [132, 4, 1],    # Present Position of 1
        [128, 4, 2],    # Present Velocity of 2
    ]
    # Probably does not work.
    # req_list = [
    #     [132, 4, 1],    # Present Position of 1
    #     [128, 4, 1],    # Present Velocity of 1
    #     [132, 4, 2],    # Present Position of 2
    #     [128, 4, 2],    # Present Velocity of 2
    # ]
    num = 100
    for i in range(3):
        start = time.time()
        for i in range(num):
            dyn.groupBulkRead(req_list)
        logging.info("elapsed_time(%s bulk-read):%s[msec]", num, time.time()*1000 - start*1000)
        start = time.time()
        for i in range(num):
            dyn.readPresentPosition(1)
            dyn.readPresentVelocity(2)
        logging.info("elapsed_time(%s 2 sync-read):%s[msec]", num, time.time()*1000 - start*1000)

if __name__ == '__main__':
    import time

    logging.basicConfig(level=logging.DEBUG)    # To see packet error messages

    DXL_IDs                      = [1, 2]
    if not hasattr(DXL_IDs, "__iter__"):
        DXL_IDs = [DXL_IDs]
    # Default baudrate : 57600
    # BAUDRATE                    = 115200
    # BAUDRATE                    = 1e6
    BAUDRATE                    = 3e6
    Dynamixel.show_list_ports()
    dyn = Dynamixel.open_ports_by_serial_number("FTAO9V7VA", BAUDRATE)
    # Check the latency timer, and set to 1 msec for fast communication
    dyn.portHandler.LATENCY_TIMER = 1
    # dyn.reboot(DXL_IDs)   # Reboot if necessary
    dyn.setRecommendedValue(DXL_IDs)
    # For safe operation, limits should be set according to the mechanical specification of the Dynamixel actuator.
    # With velocity/current/PWM control mode, position limits are not effective, but it is recommended to set them anyway.
    dyn.writeMaxPositionLimit(DXL_IDs, [3945, 2969])
    dyn.writeMinPositionLimit(DXL_IDs, [1999, 1097])


    testPositionControl(dyn, DXL_IDs)
    # testVelocityControl(dyn, DXL_IDs)
    # testReadWriteCurrent(dyn, DXL_IDs)
    # testGroupBulkRead(dyn, DXL_IDs) # NOT recomendable!
    testReadSettings(dyn, DXL_IDs)
    del dyn