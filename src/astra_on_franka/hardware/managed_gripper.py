"""Lab Dynamixel settings from franka-toy; opening a port never enables torque."""
import math
import time


class ManagedGripper:
    def __init__(self,config):
        from dynamixel_sdk import PortHandler,PacketHandler,COMM_SUCCESS
        self.config=config; self.success=COMM_SUCCESS;self.enabled=False
        self.port=PortHandler(config['device']);self.packet=PacketHandler(config['protocol_version'])
        if not self.port.openPort(): raise RuntimeError('Cannot open gripper port')
        try:
            if not self.port.setBaudRate(config['baud_rate']): raise RuntimeError('Cannot set gripper baud rate')
        except BaseException:
            self.port.closePort();raise

    def checked(self,result,error):
        if result!=self.success: raise RuntimeError(self.packet.getTxRxResult(result))
        if error: raise RuntimeError(self.packet.getRxPacketError(error))

    def read_register(self,size,address):
        value,result,error=getattr(self.packet,f'read{size}ByteTxRx')(self.port,self.config['servo_id'],address)
        self.checked(result,error);return value

    def write_register(self,size,address,value):
        result,error=getattr(self.packet,f'write{size}ByteTxRx')(self.port,self.config['servo_id'],address,value)
        self.checked(result,error)

    def read(self):
        started=time.monotonic_ns(); raw=self.read_register(4,132)
        if raw>=2**31: raw-=2**32
        hardware_error=self.read_register(1,70)
        if hardware_error: raise RuntimeError(f'Dynamixel hardware error: {hardware_error}')
        cal=self.config['calibration'];span=cal['open_position_raw']-cal['closed_position_raw']
        if not cal['closed_position_raw']-40<=raw<=cal['open_position_raw']+40:
            raise RuntimeError('Gripper encoder outside calibrated range')
        fraction=max(0.,min(1.,1-(raw-cal['closed_position_raw'])/span))
        if (time.monotonic_ns()-started)>100_000_000: raise RuntimeError('Slow gripper observation')
        return {'valid':True,'raw_position_signed':raw,'closed_fraction':fraction,
                'width_m':cal['maximum_opening_m']*(1-fraction),
                'host_received_monotonic_ns':time.monotonic_ns(),'host_received_unix_ns':time.time_ns()}

    def enable_at_current_position(self):
        if self.enabled: return
        if self.read_register(1,11)!=5: raise RuntimeError('Expected existing current-based position mode 5; no automatic mode change')
        if self.read_register(1,64)!=0: raise RuntimeError('Torque already enabled; operator must establish exclusive ownership')
        current=self.read()
        self.write_register(2,102,self.config['calibration']['grasp_current_raw'])
        self.write_register(4,116,current['raw_position_signed'] & 0xFFFFFFFF)
        self.enabled=True  # Cleanup must disable even if enable write/ack fails.
        self.write_register(1,64,1)

    def command(self,fraction):
        if not self.enabled: raise RuntimeError('Torque is not enabled')
        if not math.isfinite(fraction) or not 0<=fraction<=1: raise ValueError('Invalid fraction')
        cal=self.config['calibration']
        raw=round(cal['open_position_raw']+fraction*(cal['closed_position_raw']-cal['open_position_raw']))
        raw=max(cal['safe_closed_position_raw'],min(cal['safe_open_position_raw'],raw))
        self.write_register(4,116,raw & 0xFFFFFFFF)

    def disable(self):
        if self.enabled:
            self.write_register(1,64,0);self.enabled=False

    def close(self):
        try:self.disable()
        finally:self.port.closePort()
