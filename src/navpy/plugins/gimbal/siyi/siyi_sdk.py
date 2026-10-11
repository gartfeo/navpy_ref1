"""
Python implementation of SIYI SDK
ZR10 webpage: http://en.siyi.biz/en/Gimbal%20Camera/ZR10/overview/
Author : Mohamed Abdelkader
Email: mohamedashraf123@gmail.com
Copyright 2022

"""
import socket
from .siyi_message import *
from time import monotonic, sleep, time
import logging
from .utils import toInt
import threading

class ZR10:
    MAX_YAW_DEG = 135.0
    MIN_YAW_DEG = -135.0
    MAX_PITCH_DEG = 25.0
    MIN_PITCH_DEG = -90.0
    MAX_ZOOM = 30.0 # 10 optical * 3 digital

class SIYISDK:
    def __init__(self, server_ip="192.168.144.25", port=37260, debug=False):
        """
        Params
        --
        - server_ip [str] IP address of the camera
        - port: [int] UDP port of the camera
        """
        self._debug = debug
        if self._debug:
            d_level = logging.DEBUG
        else:
            d_level = logging.INFO
        LOG_FORMAT = ' [%(levelname)s] %(asctime)s [SIYISDK::%(funcName)s] :\t%(message)s'
        logging.basicConfig(format=LOG_FORMAT, level=d_level)
        self._logger = logging.getLogger(self.__class__.__name__)

        # Message sent to the camera
        self._out_msg = SIYIMESSAGE(debug=self._debug)

        # Message received from the camera
        self._in_msg = SIYIMESSAGE(debug=self._debug)

        self._server_ip = server_ip
        self._port = port

        self._BUFF_SIZE = 1024
        self._rcv_wait_t = 5  # Receiving wait time
        self._socket = None
        self._create_socket()

        self._current_zoom_level_lock = threading.Lock()
        self._attitude_lock = threading.Lock()
        self.resetVars()

        # Stop threads flag
        self._stop = False

        # Connection thread
        self._last_fw_seq = -1  # used to check on connection liveness
        self._conn_loop_rate = 1  # seconds

        # Gimbal info thread @ 1Hz
        self._gimbal_info_loop_rate = 1

        # Gimbal attitude thread @ 10Hz
        self._gimbal_att_loop_rate = 0.02
        self._recv_thread = None
        self._conn_thread = None
        self._g_info_thread = None
        self._g_att_thread = None
        self._init_threads()

    def resetVars(self):
        """
        Resets variables to their initial values.
        """
        self._connected = False
        self._fw_msg = FirmwareMsg()
        self._hw_msg = HardwareIDMsg()
        self._autoFocus_msg = AutoFocusMsg()
        self._manualZoom_msg = ManualZoomMsg()
        self._manualFocus_msg = ManualFocusMsg()
        self._gimbalSpeed_msg = GimbalSpeedMsg()
        self._center_msg = CenterMsg()
        self._record_msg = RecordingMsg()
        self._mountDir_msg = MountDirMsg()
        self._motionMode_msg = MotionModeMsg()
        self._funcFeedback_msg = FuncFeedbackInfoMsg()
        self._att_msg = AttitdueMsg()
        self._set_gimbal_angles_msg = SetGimbalAnglesMsg()
        self._request_data_stream_msg = RequestDataStreamMsg()
        self._request_absolute_zoom_msg = RequestAbsoluteZoomMsg()
        with self._current_zoom_level_lock:
            self._current_zoom_level_msg = CurrentZoomValueMsg()
        self._last_att_seq = -1

        return True

    def _create_socket(self):
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(self._rcv_wait_t)

    def _init_threads(self):
        self._recv_thread = threading.Thread(target=self.recvLoop, daemon=True)
        self._conn_thread = threading.Thread(target=self.connectionLoop, args=(self._conn_loop_rate,), daemon=True)
        self._g_info_thread = threading.Thread(target=self.gimbalInfoLoop, args=(self._gimbal_info_loop_rate,), daemon=True)
        self._g_att_thread = threading.Thread(target=self.gimbalAttLoop, args=(self._gimbal_att_loop_rate,), daemon=True)

    def _join_thread(self, thread):
        if thread is None:
            return
        if thread.is_alive() and thread is not threading.current_thread():
            thread.join()

    def connect(self, maxWaitTime=3.0, maxRetries=3):
        """
        Attempts to connect to the camera with retries if needed.

        Params
        --
        - maxWaitTime [float]: Maximum time to wait before giving up on connection (in seconds)
        - maxRetries [int]: Number of times to retry connecting if it fails
        """
        retries = 0
        while retries < maxRetries:
            try:
                self.resetVars()
                self._stop = False
                self._create_socket()
                self._init_threads()

                self._logger.info(f"Attempting to connect to camera, attempt {retries + 1}")
                self._recv_thread.start()
                self._conn_thread.start()
                t0 = time()

                while True:
                    if self._connected:
                        self._logger.info(f"Successfully connected to camera on attempt {retries + 1}")
                        self._g_info_thread.start()
                        self._g_att_thread.start()

                        self.requestHardwareID()
                        sleep(0.2)
                        self.requestCurrentZoomLevel()
                        sleep(0.2)
                        return True

                    if (time() - t0) > maxWaitTime and not self._connected:
                        self._logger.error("Failed to connect to camera, retrying...")
                        self.disconnect()
                        retries += 1
                        break

            except Exception as e:
                self._logger.error(f"Connection attempt {retries + 1} failed: {e}")
                self.disconnect()
                retries += 1

        self._logger.error(f"Failed to connect after {maxRetries} retries")
        return False

    def disconnect(self):
        """
        Gracefully stops all threads, disconnects, and cleans up resources.
        """
        self._logger.info("Stopping all threads and disconnecting")
        self._stop = True  # Signal threads to stop
        self._connected = False

        # Close the socket to unblock any recvfrom() calls
        if self._socket:
            try:
                self._socket.close()
            except Exception as e:
                self._logger.error(f"Error closing socket: {e}")
            finally:
                self._socket = None

        # Wait for threads to finish, if they're still alive
        self._join_thread(self._recv_thread)
        self._join_thread(self._conn_thread)
        self._join_thread(self._g_info_thread)
        self._join_thread(self._g_att_thread)

        # Reset message state. connect() recreates the socket and clears _stop.
        self.resetVars()

    def checkConnection(self):
        """
        Checks if there is a live connection to the camera by requesting the Firmware version.
        Runs in a thread at a defined frequency.
        """
        try:
            self.requestFirmwareVersion()
            sleep(0.1)
            if self._fw_msg.seq != self._last_fw_seq and len(self._fw_msg.gimbal_firmware_ver) > 0:
                self._connected = True
                self._last_fw_seq = self._fw_msg.seq
            else:
                self._connected = False
        except Exception as e:
            self._logger.error(f"Connection check failed: {e}")
            self._connected = False

    def connectionLoop(self, t):
        """
        Periodically checks connection status and resets state if disconnected.

        Params
        --
        - t [float]: message frequency in seconds
        """
        while not self._stop:
            try:
                self.checkConnection()
                sleep(t)
            except Exception as e:
                self._logger.error(f"Error in connection loop: {e}")
                self.disconnect()
                break

    def isConnected(self):
        return self._connected

    def gimbalInfoLoop(self, t):
        """
        Periodically requests gimbal info.

        Params
        --
        - t [float]: message frequency in seconds
        """
        while not self._stop:
            try:
                self.requestGimbalInfo()
                sleep(t)
            except Exception as e:
                self._logger.error(f"Error in gimbal info loop: {e}")
                self.disconnect()
                break

    def gimbalAttLoop(self, t):
        """
        Periodically requests gimbal attitude.

        Params
        --
        - t [float]: message frequency in seconds
        """
        while not self._stop:
            try:
                self.requestGimbalAttitude()
                sleep(t)
            except Exception as e:
                self._logger.error(f"Error in gimbal attitude loop: {e}")
                self.disconnect()
                break

    def sendMsg(self, msg):
        """
        Sends a message to the camera

        Params
        --
        msg [str] Message to send
        """
        try:
            if not msg:
                self._logger.error("No message to send")
                return False

            b = bytes.fromhex(msg)
            self._socket.sendto(b, (self._server_ip, self._port))
            return True
        except Exception as e:
            self._logger.error("Could not send bytes: %s", e)
            return False

    def rcvMsg(self):
        data=None
        try:
            data,addr = self._socket.recvfrom(self._BUFF_SIZE)
        except Exception as e:
            self._logger.warning("%s. Did not receive message within %s second(s)", e, self._rcv_wait_t)
        return data

    def recvLoop(self):
        self._logger.debug("Started data receiving thread")
        while( not self._stop):
            self.bufferCallback()
        self._logger.debug("Exiting data receiving thread")


    def bufferCallback(self):
        """
        Receives messages and parses its content
        """
        try:
            if self._socket is None:
                return
            buff,addr = self._socket.recvfrom(self._BUFF_SIZE)
        except OSError as e:
            if self._stop or self._socket is None:
                self._logger.debug("[bufferCallback] socket closed during shutdown")
            else:
                self._logger.error(f"[bufferCallback] {e}")
            return
        except Exception as e:
            if self._stop:
                self._logger.debug("[bufferCallback] stopping: %s", e)
            else:
                self._logger.error(f"[bufferCallback] {e}")
            return

        buff_str = buff.hex()
        self._logger.debug("Buffer: %s", buff_str)

        # 10 bytes: STX+CTRL+Data_len+SEQ+CMD_ID+CRC16
        #            2 + 1  +    2   + 2 +   1  + 2
        MINIMUM_DATA_LENGTH=10*2

        HEADER='5566'
        # Go through the buffer
        while(len(buff_str)>=MINIMUM_DATA_LENGTH):
            if buff_str[0:4]!=HEADER:
                # Remove the 1st byte and continue.
                tmp=buff_str[2:]
                buff_str=tmp
                continue

            # Now we got minimum amount of data. Check if we have enough
            # Data length, bytes are reversed, according to SIYI SDK
            low_b = buff_str[6:8] # low byte
            high_b = buff_str[8:10] # high byte
            data_len = high_b+low_b
            data_len = int('0x'+data_len, base=16)
            char_len = data_len*2

            # Check if there is enough data (including payload)
            if(len(buff_str) < (MINIMUM_DATA_LENGTH+char_len)):
                # No useful data
                buff_str=''
                break

            packet = buff_str[0:MINIMUM_DATA_LENGTH+char_len]
            buff_str = buff_str[MINIMUM_DATA_LENGTH+char_len:]

            # Finally decode the packet!
            val = self._in_msg.decodeMsg(packet)
            if val is None:
                continue

            data, data_len, cmd_id, seq = val[0], val[1], val[2], val[3]

            handler = self._CMD_DISPATCH.get(cmd_id)
            if handler is not None:
                handler(self, data, seq)
            else:
                self._logger.warning("CMD ID is not recognized")

        return

    ##################################################
    #               Request functions                #
    ##################################################
    def requestFirmwareVersion(self):
        msg = self._out_msg.firmwareVerMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestHardwareID(self):
        msg = self._out_msg.hwIdMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalAttitude(self):
        msg = self._out_msg.gimbalAttMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalInfo(self):
        msg = self._out_msg.gimbalInfoMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestFunctionFeedback(self):
        msg = self._out_msg.funcFeedbackMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestAutoFocus(self):
        msg = self._out_msg.autoFocusMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomIn(self):
        msg = self._out_msg.zoomInMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomOut(self):
        msg = self._out_msg.zoomOutMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomHold(self):
        msg = self._out_msg.stopZoomMsg()
        return self.sendMsg(msg)

    def requestAbsoluteZoom(self, level: float):
        msg = self._out_msg.absoluteZoomMsg(level)
        return self.sendMsg(msg)

    def requestCurrentZoomLevel(self):
        msg = self._out_msg.requestCurrentZoomMsg()
        return self.sendMsg(msg)

    def requestLongFocus(self):
        msg = self._out_msg.longFocusMsg()
        return self.sendMsg(msg)

    def requestCloseFocus(self):
        msg = self._out_msg.closeFocusMsg()
        return self.sendMsg(msg)

    def requestFocusHold(self):
        msg = self._out_msg.stopFocusMsg()
        return self.sendMsg(msg)

    def requestCenterGimbal(self):
        msg = self._out_msg.centerMsg()
        return self.sendMsg(msg)

    def requestGimbalSpeed(self, yaw_speed:int, pitch_speed:int):
        """
        Sends request for gimbal speed

        Params
        --
        yaw_speed [int] -100~0~100. away from zero -> fast, close to zero -> slow. Sign is for direction
        pitch_speed [int] Same as yaw_speed

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.gimbalSpeedMsg(yaw_speed, pitch_speed)
        return self.sendMsg(msg)

    def requestPhoto(self):
        msg = self._out_msg.takePhotoMsg()
        return self.sendMsg(msg)

    def requestRecording(self):
        msg = self._out_msg.recordMsg()
        return self.sendMsg(msg)

    def requestFPVMode(self):
        msg = self._out_msg.fpvModeMsg()
        return self.sendMsg(msg)

    def requestLockMode(self):
        msg = self._out_msg.lockModeMsg()
        return self.sendMsg(msg)

    def requestFollowMode(self):
        msg = self._out_msg.followModeMsg()
        return self.sendMsg(msg)

    def requestSetAngles(self, yaw_deg:float, pitch_deg:float):
        """
        Sends request to set gimbal angles

        Returns
        --
        [bool] True: success. False: fail
        """
        if self._hw_msg.cam_type_str == '':
            self._logger.error(f"Gimbal type is not yet retrieved. Check connection.")
            return False

        if self._hw_msg.cam_type_str == 'ZR10':
            if yaw_deg > ZR10.MAX_YAW_DEG:
                self._logger.warning(f"yaw_deg {yaw_deg} exceeds max {ZR10.MAX_YAW_DEG}. Setting it to max")
                yaw_deg = ZR10.MAX_YAW_DEG
            if yaw_deg < ZR10.MIN_YAW_DEG:
                self._logger.warning(f"yaw_deg {yaw_deg} exceeds min {ZR10.MIN_YAW_DEG}. Setting it to min")
                yaw_deg = ZR10.MIN_YAW_DEG
            if pitch_deg > ZR10.MAX_PITCH_DEG:
                self._logger.warning(f"pitch_deg {pitch_deg} exceeds max {ZR10.MAX_PITCH_DEG}. Setting it to max")
                pitch_deg = ZR10.MAX_PITCH_DEG
            if pitch_deg < ZR10.MIN_PITCH_DEG:
                self._logger.warning(f"pitch_deg {pitch_deg} exceeds min {ZR10.MIN_PITCH_DEG}. Setting it to min")
                pitch_deg = ZR10.MIN_PITCH_DEG
        else:
            self._logger.warning(f"Camera not supported. Setting angles to zero")
            return False

        msg = self._out_msg.setGimbalAttitude(int(yaw_deg*10), int(pitch_deg*10))
        return self.sendMsg(msg)

    def requestDataStreamAttitude(self, freq: int):
        msg = self._out_msg.dataStreamMsg(1, freq)
        return self.sendMsg(msg)

    def requestDataStreamLaser(self, freq: int):
        msg = self._out_msg.dataStreamMsg(2, freq)
        return self.sendMsg(msg)

    ####################################################
    #                Parsing functions                 #
    ####################################################
    def parseFirmwareMsg(self, msg:str, seq:int):
        try:
            self._fw_msg.gimbal_firmware_ver= msg[8:16]
            self._fw_msg.seq=seq
            self._logger.debug("Firmware version: %s", self._fw_msg.gimbal_firmware_ver)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseHardwareIDMsg(self, msg:str, seq:int):
        try:
            self._hw_msg.seq=seq
            self._hw_msg.id = bytes.fromhex(msg).decode('ascii', errors='ignore').rstrip('\x00')
            self._logger.debug("Hardware ID: %s", self._hw_msg.id)
            cam_id = self._hw_msg.id[:2].upper()
            try:
                self._hw_msg.cam_type_str = self._hw_msg.CAM_DICT[cam_id]
            except Exception as e:
                self._logger.error(f"Camera not recognized. Key: {cam_id}")
                self._logger.error("Camera not recognized Error %s", e)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseAttitudeMsg(self, msg:str, seq:int):
        try:
            with self._attitude_lock:
                self._att_msg.seq=seq
                self._att_msg.stamp=time()
                self._att_msg.yaw = toInt(msg[2:4]+msg[0:2]) /10.
                self._att_msg.pitch = toInt(msg[6:8]+msg[4:6]) /10.
                self._att_msg.roll = toInt(msg[10:12]+msg[8:10]) /10.
                self._att_msg.yaw_speed = toInt(msg[14:16]+msg[12:14]) /10.
                self._att_msg.pitch_speed = toInt(msg[18:20]+msg[16:18]) /10.
                self._att_msg.roll_speed = toInt(msg[22:24]+msg[20:22]) /10.

            self._logger.debug("(yaw, pitch, roll= (%s, %s, %s)",
                                    self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)
            self._logger.debug("(yaw_speed, pitch_speed, roll_speed= (%s, %s, %s)",
                                    self._att_msg.yaw_speed, self._att_msg.pitch_speed, self._att_msg.roll_speed)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalInfoMsg(self, msg:str, seq:int):
        try:
            self._record_msg.seq=seq
            self._mountDir_msg.seq=seq
            self._motionMode_msg.seq=seq

            self._record_msg.state = int('0x'+msg[6:8], base=16)
            self._motionMode_msg.mode = int('0x'+msg[8:10], base=16)
            self._mountDir_msg.dir = int('0x'+msg[10:12], base=16)

            self._logger.debug("Recording state %s", self._record_msg.state)
            self._logger.debug("Mounting direction %s", self._mountDir_msg.dir)
            self._logger.debug("Gimbal motion mode %s", self._motionMode_msg.mode)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseAutoFocusMsg(self, msg:str, seq:int):
        try:
            self._autoFocus_msg.seq=seq
            self._autoFocus_msg.success = bool(int('0x'+msg, base=16))
            self._logger.debug("Auto focus success: %s", self._autoFocus_msg.success)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseZoomMsg(self, msg:str, seq:int):
        try:
            self._manualZoom_msg.seq=seq
            self._manualZoom_msg.level = int('0x'+msg[2:4]+msg[0:2], base=16) /10.
            self._logger.debug("Zoom level %s", self._manualZoom_msg.level)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseManualFocusMsg(self, msg:str, seq:int):
        try:
            self._manualFocus_msg.seq=seq
            self._manualFocus_msg.success = bool(int('0x'+msg, base=16))
            self._logger.debug("Manual  focus success: %s", self._manualFocus_msg.success)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalSpeedMsg(self, msg:str, seq:int):
        try:
            self._gimbalSpeed_msg.seq=seq
            self._gimbalSpeed_msg.success = bool(int('0x'+msg, base=16))
            self._logger.debug("Gimbal speed success: %s", self._gimbalSpeed_msg.success)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalCenterMsg(self, msg:str, seq:int):
        try:
            self._center_msg.seq=seq
            self._center_msg.success = bool(int('0x'+msg, base=16))
            self._logger.debug("Gimbal center success: %s", self._center_msg.success)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseFunctionFeedbackMsg(self, msg:str, seq:int):
        try:
            self._funcFeedback_msg.seq=seq
            self._funcFeedback_msg.info_type = int('0x'+msg, base=16)
            self._logger.debug("Function Feedback Code: %s", self._funcFeedback_msg.info_type)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseSetGimbalAnglesMsg(self, msg:str, seq:int):
        try:
            self._set_gimbal_angles_msg.seq=seq
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseAbsoluteZoomMsg(self, msg: str, seq: int):
        try:
            self._request_absolute_zoom_msg.seq = seq
            if msg:
                self._request_absolute_zoom_msg.success = bool(int('0x'+msg, base=16))
            else:
                self._request_absolute_zoom_msg.success = True
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseRequestStreamMsg(self, msg:str, seq:int):
        try:
            self._request_data_stream_msg.seq=seq
            self._request_data_stream_msg.data_type = int('0x'+msg, base=16)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseCurrentZoomLevelMsg(self, msg: str, seq: int):
        try:
            int_part = int('0x'+msg[0:2], base=16)
            float_part = int('0x'+msg[2:4], base=16)
            level = int_part + (float_part/10)
            received_monotonic_s = monotonic()
            with self._current_zoom_level_lock:
                self._current_zoom_level_msg.seq = seq
                self._current_zoom_level_msg.level = level
                self._current_zoom_level_msg.receipt_monotonic_s = received_monotonic_s
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    # Dispatch table: COMMAND → parse method (OCP: new commands are additive)
    _CMD_DISPATCH = {
        COMMAND.ACQUIRE_FW_VER:      parseFirmwareMsg,
        COMMAND.ACQUIRE_HW_ID:       parseHardwareIDMsg,
        COMMAND.ACQUIRE_GIMBAL_INFO: parseGimbalInfoMsg,
        COMMAND.ACQUIRE_GIMBAL_ATT:  parseAttitudeMsg,
        COMMAND.FUNC_FEEDBACK_INFO:  parseFunctionFeedbackMsg,
        COMMAND.GIMBAL_SPEED:        parseGimbalSpeedMsg,
        COMMAND.AUTO_FOCUS:          parseAutoFocusMsg,
        COMMAND.MANUAL_FOCUS:        parseManualFocusMsg,
        COMMAND.MANUAL_ZOOM:         parseZoomMsg,
        COMMAND.CENTER:              parseGimbalCenterMsg,
        COMMAND.SET_GIMBAL_ATTITUDE: parseSetGimbalAnglesMsg,
        COMMAND.SET_DATA_STREAM:     parseRequestStreamMsg,
        COMMAND.ABSOLUTE_ZOOM:       parseAbsoluteZoomMsg,
        COMMAND.CURRENT_ZOOM_VALUE:  parseCurrentZoomLevelMsg,
    }

    ##################################################
    #                   Get functions                #
    ##################################################
    def getAttitude(self):
        with self._attitude_lock:
            return(self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)

    def getAttitudeSample(self):
        with self._attitude_lock:
            return (
                self._att_msg.seq,
                self._att_msg.stamp,
                self._att_msg.yaw,
                self._att_msg.pitch,
                self._att_msg.roll,
            )

    def getAttitudeSpeed(self):
        with self._attitude_lock:
            return(self._att_msg.yaw_speed, self._att_msg.pitch_speed, self._att_msg.roll_speed)

    def getFirmwareVersion(self):
        return(self._fw_msg.gimbal_firmware_ver)

    def getHardwareID(self):
        return(self._hw_msg.id)

    def getCameraTypeString(self):
        return(self._hw_msg.cam_type_str)

    def getRecordingState(self):
        return(self._record_msg.state)

    def getMotionMode(self):
        return(self._motionMode_msg.mode)

    def getMountingDirection(self):
        return(self._mountDir_msg.dir)

    def getFunctionFeedback(self):
        return(self._funcFeedback_msg.info_type)

    def getZoomLevel(self):
        return(self._manualZoom_msg.level)

    def getCurrentZoomLevel(self):
        with self._current_zoom_level_lock:
            return(self._current_zoom_level_msg.level)

    def getCurrentZoomLevelSample(self):
        with self._current_zoom_level_lock:
            return (
                self._current_zoom_level_msg.seq,
                self._current_zoom_level_msg.level,
                self._current_zoom_level_msg.receipt_monotonic_s,
            )

    def getCenteringFeedback(self):
        return(self._center_msg.success)

    def getDataStreamFeedback(self):
        return(self._request_data_stream_msg.data_type)

    #################################################
    #                 Set functions                 #
    #################################################
    def setGimbalRotation(self, yaw, pitch, err_thresh=1.0, kp=4):
        """
        Sets gimbal attitude angles yaw and pitch in degrees

        Params
        --
        yaw: [float] desired yaw in degrees
        pitch: [float] desired pitch in degrees
        err_thresh: [float] acceptable error threshold, in degrees, to stop correction
        kp [float] proportional gain
        """
        if pitch > ZR10.MAX_PITCH_DEG or pitch < ZR10.MIN_PITCH_DEG:
            self._logger.error(
                "Desired pitch is outside controllable range %.1f~%.1f",
                ZR10.MIN_PITCH_DEG,
                ZR10.MAX_PITCH_DEG,
            )
            return

        if yaw > ZR10.MAX_YAW_DEG or yaw < ZR10.MIN_YAW_DEG:
            self._logger.error(
                "Desired yaw is outside controllable range %.1f~%.1f",
                ZR10.MIN_YAW_DEG,
                ZR10.MAX_YAW_DEG,
            )
            return

        th = err_thresh
        gain = kp
        while(True):
            self.requestGimbalAttitude()
            if self._att_msg.seq==self._last_att_seq:
                self._logger.info("Did not get new attitude msg")
                self.requestGimbalSpeed(0,0)
                continue

            self._last_att_seq = self._att_msg.seq

            yaw_err = -yaw + self._att_msg.yaw # NOTE for some reason it's reversed!!
            pitch_err = pitch - self._att_msg.pitch

            self._logger.debug("yaw_err= %s", yaw_err)
            self._logger.debug("pitch_err= %s", pitch_err)

            if (abs(yaw_err) <= th and abs(pitch_err)<=th):
                self.requestGimbalSpeed(0, 0)
                self._logger.info("Goal rotation is reached")
                break

            y_speed_sp = max(min(100, int(gain*yaw_err)), -100)
            p_speed_sp = max(min(100, int(gain*pitch_err)), -100)
            self._logger.debug("yaw speed setpoint= %s", y_speed_sp)
            self._logger.debug("pitch speed setpoint= %s", p_speed_sp)
            self.requestGimbalSpeed(y_speed_sp, p_speed_sp)

            sleep(0.1) # command frequency
