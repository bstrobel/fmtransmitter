import argparse
import board
import digitalio
import busio
import math
import time
from enum import Enum

I2C_ADDR = 0x2c

REG_SYSTEM=0x00
REG_SYSTEM_SWRST_BIT = 7 # Reset all registers to default values
class SWRST(Enum):
    KEEP = 0 # Keep the current value.
    RESET = 1 # Reset to default values.
REG_SYSTEM_RECAL_BIT = 6 # Reset the state to initial states and recalibrate all blocks.
class RECAL(Enum):
    NO_RECAL = 0 # No reset. FSM runs normally.
    DO_RECAL = 1 # Reset the FSM. After this bit is de-asserted, FSM will go through all the power up and calibration sequence.
REG_SYSTEM_TXREQ_BIT = 5 # Transmission request
class TXREQ(Enum):
    IDLE = 0 # Stay in IDLE mode.
    TRANSMIT = 1 # Enter Transmit mode.
REG_SYSTEM_MONO_BIT = 4 # Force MONO mode for transmission
class MONO(Enum):
    STEREO = 0 # Stereo mode.
    MONO = 1 # MONO mode.
REG_SYSTEM_MUTE_BIT = 3 # Audio Mute enable
class MUTE(Enum):
    OFF = 0 # Not Mute
    ON = 1 # Mute
REG_SYSTEM_RDSRDY_BIT = 2 # RDS transmitting ready
class RDSRDY(Enum):
    IDLE = 0
    DOSEND = 1
REG_SYSTEM_CH_BIT = 0 # Highest 2 bits of 10-bit channel index: Channel freq is (76+CH*0.05) MHz
REG_SYSTEM_CH_88_8MHZ = 1 # default

REG_CH1=0x01 # Lower 8 bits of 10-bit Channel index. Channel used for TX
REG_CH1_CH_88_8MHZ = 0 # default

REG_GPLT=0x02
REG_GPLT_TC_BIT = 7 # Pre-emphasis time constant
class TC(Enum):
    TC_50US = 0
    TC_75US = 1 # default
REG_GPLT_PRIVEN_BIT = 6 # Enable the privacy mode (audio scramble and RDS encryption)
class PRIVEN(Enum):
    PRIVEN_ENABLE = 1
    PRIVEN_DISABLE = 0 # default
REG_GPLT_T1MSEL_BIT = 4 # Selection of 1 minute time for PA off when no audio. The real time is (58+t1m_sel) seconds
class GPLT_T1MSEL(Enum):
    T1MSEL_58s = 0b00
    T1MSEL_59s = 0b01
    T1MSEL_60s = 0b10 #default
    T1MSEL_INFINITY = 0b11
REG_GPLT_GAINTXPLT_BIT = 0 # Gain of TX pilot to adjust pilot frequency deviation. Refer to peak frequency deviation of MPX signal when audio input is full scale.
class GPLT_GAINTXPLT(Enum):
    GAINTXPLT_7PCT = 0b0111
    GAINTXPLT_8PCT = 0b1000
    GAINTXPLT_9PCT = 0b1001 # default
    GAINTXPLT_10PCT = 0b1010
    GAINTXPLT_INVALID = 0

REG_XTL=0x03
REG_XTL_XINJ_BIT = 6 # Select the reference clock source
class XTL_XINJ(Enum):
    USE_CRYSTAL = 0 # default - Use crystal on XTAL1/XTAL2
    EXT_CLOCK_ON_XTAL1 = 1 # Inject digital clock from XTAL1
    EXT_SINE_CLOCK_ON_XTAL1 = 2 # Single end sine-wave injection on XTAL1
    EXT_DIFFERENTIAL_SINE_CLOCK = 3 # Differential sine-wave injection on XTAL1/2
REG_XTL_XISEL_BIT = 0 # Crystal oscillator current control. 6.25uA*XISEL[5:0], 0-400uA when use crystal on XTAL1/XTAL2.
REG_XTL_XISEL_CURRENT_100uA = 0b010000 # default

REG_VGA=0x04
REG_VGA_XSEL_BIT=7 # Crystal frequency selection
class VGA_XSEL(Enum):
    XTAL_12MHZ = 0
    XTAL_24MHZ = 1 # default
REG_VGA_GVGA_BIT = 4 # TX input buffer gain (dB), actual value depends on RIN, see documentation
class VGA_GVGA_LEVEL(Enum):
    L1 = 0
    L2 = 1
    L3 = 2
    L4 = 3 # default: 0 dB - RIN = 20 (default RIN)
    L5 = 4
    L6 = 5
    INVALID = 127
VGA_GAIN_MX = [
    ['3','-3', '-9', '-15'],
    ['6', '0', '-6', '-9'],
    ['9', '3', '-3', '-9'],
    ['12', '6', '0', '-6'],
    ['15', '9', '3', '-3'],
    ['18', '12', '6', '0']
]
REG_VGA_GDB_BIT = 2
class VGA_GDB(Enum):
    GDB_0DB = 0b00
    GDB_1DB = 0b01
    GDB_2DB = 0b10
    GDB_RESERVED = 0b11
REG_VGA_RIN_BIT = 0 # TX mode input impedance for both L/R channels.
class VGA_R_IN(Enum):
    RIN_5KOHM = 0b00
    RIN_10KOHM = 0b01
    RIN_20KOHM = 0b10 # default
    RIN_40KOHM = 0b11
    


REG_CID1=0x05
REG_CID1_CID0_BIT = 5 # reserved
REG_CID1_CID1_BIT = 2 # Chip ID for product family
REG_CID1_CID1_FM_FAMILY = 0 # must be 0, other values reserved
REG_CID1_CID2_BIT = 0 # Minor revision 00 -> rev1, ..., 11 -> rev4

REG_CID2=0x06
REG_CID2_CID3_BIT = 4 # Chip ID for product ID
REG_CID2_CID3_QN8027 = 0b0100 # all other values reserved
REG_CID2_CID4_BIT = 0 # Major revision 00 -> rev1, ...

REG_STATUS=0x07
REG_STATUS_AUDPK_BIT = 4 # Audio peak value at ADC input is aud_pk[3:0]*45mV
REG_STATUS_RDSUPD_BIT = 3 # RDS TX: To transmit the 8 bytes in RDS0~RDS7, the user should toggle the register bit RDSRDY. Then the chip internally fetches these bytes after completing transmitting the current group. Once the chip has internally fetched these bytes, it will toggle this bit, and the user can write in another group.
class RDSUPD(Enum):
    OFF = 0
    ON = 1
REG_STATUS_FSM_BIT = 0 # Top FSM state code
class FSM(Enum):
    RESET = 0b000 # in RESET state
    CALI = 0b001 # in CALI state
    IDLE = 0b010 # in IDLE state
    TXRSTB = 0b011 # in TX_RSTB state
    PA_CALIB = 0b100 # PA calibration
    TRANSMIT = 0b101 # Transmit
    PA_OFF = 0b110 # PA_OFF
    RESERVED = 0b111 # Reserved

# RDS data byte0 to be sent: Data written into RDSD0~RDSD7 can not be sent out if user didn’t toggle RDSRDY to allow the data to be loaded into the internal transmitting buffer.
REG_RDS0=0x08
REG_RDS1=0x09
REG_RDS2=0x0A
REG_RDS3=0x0B
REG_RDS4=0x0C
REG_RDS5=0x0D
REG_RDS6=0x0E
REG_RDS7=0x0F

REG_PAC=0x10
REG_PAC_TXPDCLR_BIT = 7 # TX aud_pk clear signal: Audio peak value is max-hold and stored in aud_pk[3:0]. Once TXPD_CLR is toggled, the aud_pk value is cleared and restarted again.
REG_PAC_PATRGT_BIT = 0 # PA output power target is 0.62*PA_TRGT+71dBu. Valid values according to manual are 20-75. Other values seem to work as well.
REG_PAC_PATRGT_MAX = 0b1111111 # default

REG_FDEV=0x11 # Specify total TX frequency deviation: TX frequency deviation = 0.58 kHz*TX_FDEV
REG_FDEV_74_82KHZ = 0b10000001 # default 74.82kHz

REG_RDS=0x12
REG_RDS_RDSEN_BIT = 7 # RDS enable
class RDS(Enum):
    DISABLED = 0
    ENABLED = 1
REG_RDS_RDSFDEV_BIT = 0 # Specify RDS frequency deviation: RDS frequency deviation = 0.35KHz*RDSFDEV.
REG_RDS_RDSFDEV_2_1KHZ = 0b0000110 # default 2.1kHz

# default values for initializing the chip
DEFAULT_RIN = VGA_R_IN.RIN_10KOHM
DEFAULT_VGA_LEVEL = VGA_GVGA_LEVEL.L2
DEFAULT_GDB = VGA_GDB.GDB_0DB
DEFAULT_XTAL = VGA_XSEL.XTAL_12MHZ
DEFAULT_XINJ = XTL_XINJ.USE_CRYSTAL
DEFAULT_XISEL = REG_XTL_XISEL_CURRENT_100uA
DEFAULT_TC = TC.TC_75US
DEFAULT_T1MSEL = GPLT_T1MSEL.T1MSEL_INFINITY
DEFAULT_GAINTXPLT = GPLT_GAINTXPLT.GAINTXPLT_9PCT
DEFAULT_PRIVEN = PRIVEN.PRIVEN_DISABLE
DEFAULT_PA_TARGET = 50
DEFAULT_FREQ = 90.9

class qn8027:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.state = bytearray(0x13)
        self.get_state()

    def send_cmd(self,reg,val):
        self.i2c.writeto(I2C_ADDR, bytes([reg,val]))

    def get_state(self):
        """Reads the whole set of registers into self.state byte array and parses the values into class member variables"""
        self.i2c.writeto_then_readfrom(I2C_ADDR, bytes([REG_SYSTEM]), self.state)

        reg_system_val = self.state[REG_SYSTEM]
        self.swrst = SWRST((reg_system_val >> REG_SYSTEM_SWRST_BIT) & 0b00000001)
        self.recal = RECAL((reg_system_val >> REG_SYSTEM_RECAL_BIT) & 0b00000001)
        self.txreq = TXREQ((reg_system_val >> REG_SYSTEM_TXREQ_BIT) & 0b00000001)
        self.mono = MONO((reg_system_val >> REG_SYSTEM_MONO_BIT) & 0b00000001)
        self.mute = MUTE((reg_system_val >> REG_SYSTEM_MUTE_BIT) & 0b00000001)
        self.rdsrdy = RDSRDY((reg_system_val >> REG_SYSTEM_RDSRDY_BIT) & 0b00000001)
        self.freqhi = (reg_system_val & 0b00000011) * 256 * 0.05
        self.freqlo = self.state[REG_CH1]  * 0.05
        self.freq = self.freqhi + self.freqlo + 76

        reg_gplt_val = self.state[REG_GPLT]
        self.tc = TC((reg_gplt_val >> REG_GPLT_TC_BIT) & 0b00000001)
        self.priv_en = PRIVEN((reg_gplt_val >> REG_GPLT_PRIVEN_BIT) & 0b00000001)
        self.t1m_sel = GPLT_T1MSEL((reg_gplt_val >> REG_GPLT_T1MSEL_BIT) & 0b00000011)
        gain_txplt_val = (reg_gplt_val & 0b00001111)
        try:
            self.gain_txplt = GPLT_GAINTXPLT(gain_txplt_val)
        except ValueError:
            self.gain_txplt = GPLT_GAINTXPLT.GAINTXPLT_INVALID
        reg_xtl_val = self.state[REG_XTL]
        self.xinj = XTL_XINJ((reg_xtl_val >> REG_XTL_XINJ_BIT) & 0b00000011)
        self.xisel = (reg_xtl_val & 0b0011111) * 6.25

        reg_vga_val = self.state[REG_VGA]
        self.xsel = VGA_XSEL((reg_vga_val >> REG_VGA_XSEL_BIT) & 0b00000001)
        self.rin = VGA_R_IN(reg_vga_val & 0b00000011)
        self.gdb = VGA_GDB((reg_vga_val >> REG_VGA_GDB_BIT) & 0b00000011)
        try:
            self.gvga = VGA_GVGA_LEVEL((reg_vga_val >> REG_VGA_GVGA_BIT) & 0b00000111)
        except:
            self.gvga = VGA_GVGA_LEVEL.INVALID
            print(f'WARNING: Invalid GVGA value: {(reg_vga_val >> REG_VGA_GVGA_BIT) & 0b00000111:03b}')
        try:
            self.tx_buffer_gain = VGA_GAIN_MX[self.gvga.value][self.rin.value]
        except:
            self.tx_buffer_gain = 'INVALID'
        
        reg_cid1_val = self.state[REG_CID1]
        reg_cid2_val = self.state[REG_CID2]
        if (reg_cid1_val >> REG_CID1_CID1_BIT) & 0b00000111 == REG_CID1_CID1_FM_FAMILY:
            self.cid_product_family = 'FM'
        else:
            self.cid_product_family = 'UNKNOWN PRODUCT FAMILY'
        if (reg_cid2_val >> REG_CID2_CID3_BIT) & 0b00001111 == REG_CID2_CID3_QN8027:
            self.cid_product = 'QN8027'
        else:
            self.cid_product = 'UNKNOWN PRODUCT'
        self.cid_minor_revision = (reg_cid1_val & 0b00000011) + 1
        self.cid_major_revision = (reg_cid2_val & 0b00001111) + 1
        # my chip has revision 5...
        # if (cid_major_revision > 4):
            # cid_major_revision = 'RESERVED'

        reg_status_val = self.state[REG_STATUS]
        self.fsmstate = FSM(reg_status_val & 0b00000111)
        self.rds_upd = RDSUPD((reg_status_val >> REG_STATUS_RDSUPD_BIT) & 0b00000001)
        self.aud_pk = ((reg_status_val >> REG_STATUS_AUDPK_BIT) & 0b00001111) * 45

        reg_pac_val = self.state[REG_PAC]
        self.txpd_clr = (reg_pac_val >> REG_PAC_TXPDCLR_BIT) & 0b00000001
        self.pa_trgt = 0.62 * (reg_pac_val & 0b01111111) + 71

        reg_fdev_val = self.state[REG_FDEV]
        self.tx_fdev = 0.58 * reg_fdev_val

        reg_rds_val = self.state[REG_RDS]
        self.rdsen = RDS((reg_rds_val >> REG_RDS_RDSEN_BIT) & 0b00000001)
        self.rdsfdev = 0.35 * (reg_rds_val & 0b01111111)

    def dbg_print_reg_system(self,hexvals=False):
        if (hexvals):
            print(f'[REG_SYSTEM] 0x{self.state[REG_SYSTEM]:02x} {self.state[REG_SYSTEM]:2d} 0b{self.state[REG_SYSTEM]:08b}')
        print(f'[REG_SYSTEM] swrst={self.swrst.name} recal={self.recal.name} txreq={self.txreq.name} mono={self.mono.name} mute={self.mute.name} rdsrdy={self.rdsrdy.name}')

    def dbg_print_freq(self,hexvals=False):
        print(f'[FREQ] {str(self.freq)}MHz = 76 + freqhi={str(self.freqhi)} + freqlo={str(self.freqlo)}')
    
    def dbg_print_reg_gplt(self,hexvals=False):
        if (hexvals):
            print(f'[REG_GPLT] 0x{self.state[REG_GPLT]:02x} {self.state[REG_GPLT]:2d} 0b{self.state[REG_GPLT]:08b}')
        print(f'[REG_GPLT] tc={self.tc.name} priv_en={self.priv_en.name} t1m_sel={self.t1m_sel.name} gain_txplt={self.gain_txplt.name}')

    def dbg_print_reg_xtl(self,hexvals=False):
        if (hexvals):
            print(f'[REG_XTL] 0x{self.state[REG_XTL]:02x} {self.state[REG_XTL]:2d} 0b{self.state[REG_XTL]:08b}')
        print(f'[REG_XTL] xinj={self.xinj.name} xisel={self.xisel}uA')

    def dbg_print_reg_vga(self,hexvals=False):
        if (hexvals):
            print(f'[REG_VGA] 0x{self.state[REG_VGA]:02x} {self.state[REG_VGA]:2d} 0b{self.state[REG_VGA]:08b}')
        print(f'[REG_VGA] xsel={self.xsel.name} tx_buffer_gain={self.tx_buffer_gain}dB gdb={self.gdb.name} rin={self.rin.name} gvga={self.gvga.name}')

    def dbg_print_cid(self,hexvals=False):
        if (hexvals):
            print(f'[REG_CID1] 0x{self.state[REG_CID1]:02x} {self.state[REG_CID1]:2d} 0b{self.state[REG_CID1]:08b}')
            print(f'[REG_CID2] 0x{self.state[REG_CID2]:02x} {self.state[REG_CID2]:2d} 0b{self.state[REG_CID2]:08b}')
        print(f'[CID] familiy={self.cid_product_family} product={self.cid_product} revision={self.cid_major_revision}.{self.cid_minor_revision}')

    def dbg_print_reg_status(self,hexvals=False):
        if (hexvals):
            print(f'[REG_STATUS] 0x{self.state[REG_STATUS]:02x} {self.state[REG_STATUS]:2d} 0b{self.state[REG_STATUS]:08b}')
        print(f'[REG_STATUS] fsmstate={self.fsmstate.name} rds_upd={self.rds_upd.name} aud_pk={self.aud_pk}mV')

    def dbg_print_reg_pac(self,hexvals=False):
        if (hexvals):
            print(f'[REG_PAC] 0x{self.state[REG_PAC]:02x} {self.state[REG_PAC]:2d} 0b{self.state[REG_PAC]:08b}')
        print(f'[REG_PAC] txpd_clr={self.txpd_clr} pa_trgt={self.pa_trgt}dBu')

    def dbg_print_reg_fdev(self,hexvals=False):
        if (hexvals):
            print(f'[REG_FDEV] 0x{self.state[REG_FDEV]:02x} {self.state[REG_FDEV]:2d} 0b{self.state[REG_FDEV]:08b}')
        print(f'[REG_FDEV] tx_fdev={self.tx_fdev}kHz')

    def dbg_print_reg_rds(self,hexvals=False):
        if (hexvals):
            print(f'[REG_RDS] 0x{self.state[REG_RDS]:02x} {self.state[REG_RDS]:2d} 0b{self.state[REG_RDS]:08b}')
        print(f'[REG_RDS] rdsen={self.rdsen.name} rdsfdev={self.rdsfdev}kHz')

    def dbg_print_state(self, hexvals=False):
        self.get_state()
        self.dbg_print_reg_system(hexvals)
        self.dbg_print_freq(hexvals)
        self.dbg_print_reg_gplt(hexvals)
        self.dbg_print_reg_xtl(hexvals)
        self.dbg_print_reg_vga(hexvals)
        self.dbg_print_cid(hexvals)
        self.dbg_print_reg_status(hexvals)
        self.dbg_print_reg_pac(hexvals)
        self.dbg_print_reg_fdev(hexvals)
        self.dbg_print_reg_rds(hexvals)

    def send_org_init(self):
        """Sends the same commands as the original microcontroller did"""
        self.send_cmd(REG_XTL,0x3e)
        # self.send_cmd(REG_VGA,0x36) # 12MHz xtal
        self.send_cmd(REG_VGA,0xa6) # 24MHz xtal
        self.send_cmd(REG_SYSTEM,0x41) # SYSTEM.RECAL=1 - Reset the FSM. After this bit is de-asserted, FSM will go through all the power up and calibration sequence.
        self.send_cmd(REG_SYSTEM,0x01) # SYSTEM: No reset, no recal, idle (no txreq), stereo, no mute, no rdsrdy
        # self.send_cmd(0x18,0xe4) # undocumented command
        # self.send_cmd(0x1b,0xf0) # undocumented command
        self.send_cmd(REG_GPLT,0xb9)
        self.send_cmd(REG_CH1,0x74)
        self.send_cmd(REG_SYSTEM,0x21) # SYSTEM: No reset, no recal, TXREQ, stereo, no mute, no rdsrdy
        self.get_state()

    def reset_chip(self):
        self.send_cmd(REG_SYSTEM,1 << REG_SYSTEM_SWRST_BIT)

    def start_fsm_recalibration(self):
        """Trigger FSM recalibration"""
        self.get_state()
        self.send_cmd(REG_SYSTEM, 1 << REG_SYSTEM_RECAL_BIT)
        time.sleep(0.1)
        self.send_cmd(REG_SYSTEM, self.state[REG_SYSTEM] & ~(1 << REG_SYSTEM_RECAL_BIT))

    def set_freq(self, freqMHz):
        self.get_state()
        channel = int(math.floor(((freqMHz*1000000-76000000))/50000))
        fHiBits = (channel >> 8) & 255
        fLoBits = channel & 255
        reg_system_new = (self.state[REG_SYSTEM] & 0b11111100) | fHiBits
        reg_ch1_new = fLoBits
        self.send_cmd(REG_CH1,reg_ch1_new)
        self.send_cmd(REG_SYSTEM,reg_system_new)

    def set_enable_tx(self, enableTX=True):
        self.get_state()
        if enableTX:
            reg_system_new = self.state[REG_SYSTEM] | (1 << REG_SYSTEM_TXREQ_BIT)
        else:
            reg_system_new = self.state[REG_SYSTEM] & ~(1 << REG_SYSTEM_TXREQ_BIT)
        self.send_cmd(REG_SYSTEM,reg_system_new)

    def set_enable_stereo(self, stereoOn=True):
        self.get_state()
        if stereoOn:
            reg_system_new = self.state[REG_SYSTEM] & ~(1 << REG_SYSTEM_MONO_BIT)
        else:
            reg_system_new = self.state[REG_SYSTEM] | (1 << REG_SYSTEM_MONO_BIT)
        self.send_cmd(REG_SYSTEM,reg_system_new)

    def set_enable_mute(self, muteOn=True):
        self.get_state()
        if muteOn:
            reg_system_new = self.state[REG_SYSTEM] | (1 << REG_SYSTEM_MUTE_BIT)
        else:
            reg_system_new = self.state[REG_SYSTEM] & ~(1 << REG_SYSTEM_MUTE_BIT)
        self.send_cmd(REG_SYSTEM,reg_system_new)

    def set_vga(self, rin=VGA_R_IN.RIN_20KOHM, gvga=VGA_GVGA_LEVEL.L3, gdb=VGA_GDB.GDB_0DB):
        """Sets the values for the variable gain amplifier for the audio input stage"""
        self.get_state()
        reg_vga_new = (self.state[REG_VGA] & (0b10000000)) | \
            gvga.value << REG_VGA_GVGA_BIT | gdb.value << REG_VGA_GDB_BIT | rin.value << REG_VGA_RIN_BIT
        self.send_cmd(REG_VGA, reg_vga_new)

    def set_rin(self, rin):
        self.get_state()
        reg_vga_new = (self.state[REG_VGA] & (0b11111100)) | rin.value << REG_VGA_RIN_BIT
        self.send_cmd(REG_VGA, reg_vga_new)

    def set_gvga(self, gvga):
        self.get_state()
        reg_vga_new = (self.state[REG_VGA] & (0b10001111)) | gvga.value << REG_VGA_GVGA_BIT
        self.send_cmd(REG_VGA, reg_vga_new)

    def set_gdb(self, gdb):
        self.get_state()
        reg_vga_new = (self.state[REG_VGA] & (0b11110011)) | gdb.value << REG_VGA_GDB_BIT
        self.send_cmd(REG_VGA, reg_vga_new)

    def set_xtal(self, xtal=VGA_XSEL.XTAL_24MHZ, xinj=XTL_XINJ.USE_CRYSTAL, xisel=REG_XTL_XISEL_CURRENT_100uA):
        self.get_state()
        reg_vga_new = (self.state[REG_VGA] & (0b01111111)) | xtal.value << REG_VGA_XSEL_BIT
        self.send_cmd(REG_VGA, reg_vga_new)
        xisel &= 0b111111
        reg_xtl_new = xinj.value << REG_XTL_XINJ_BIT | xisel
        self.send_cmd(REG_XTL, reg_xtl_new)

    def set_gplt(self, tc=TC.TC_75US, t1m1=GPLT_T1MSEL.T1MSEL_INFINITY, gain_txplt=GPLT_GAINTXPLT.GAINTXPLT_9PCT, priv_en=PRIVEN.PRIVEN_DISABLE):
        reg_gplt_new = tc.value << REG_GPLT_TC_BIT | \
            t1m1.value << REG_GPLT_T1MSEL_BIT | \
            gain_txplt.value << REG_GPLT_GAINTXPLT_BIT | \
            priv_en.value << REG_GPLT_PRIVEN_BIT
        self.send_cmd(REG_GPLT,reg_gplt_new)
    
    def set_pac(self, pa_target=REG_PAC_PATRGT_MAX):
        self.send_cmd(REG_PAC,pa_target & 0b01111111)

    def reset_aud_pk(self):
        self.get_state()
        self.send_cmd(REG_PAC, 0b10000000 | self.state[REG_PAC])
        time.sleep(0.01)
        self.send_cmd(REG_PAC, 0b01111111 & self.state[REG_PAC])


    def default_init(self):
        self.reset_chip()
        self.set_vga(rin=DEFAULT_RIN, gdb=DEFAULT_GDB, gvga=DEFAULT_VGA_LEVEL)
        self.set_xtal(xtal=DEFAULT_XTAL, xinj=DEFAULT_XINJ, xisel=DEFAULT_XISEL)
        self.set_gplt(DEFAULT_TC, DEFAULT_T1MSEL, DEFAULT_GAINTXPLT, DEFAULT_PRIVEN)
        self.set_pac(DEFAULT_PA_TARGET)
        self.set_freq(DEFAULT_FREQ)
        self.start_fsm_recalibration()
        self.set_enable_tx()

def main():
    p = argparse.ArgumentParser()
    # p.add_argument('-v','--verbose', action='store_true', help='verbose mode')
    ginit = p.add_mutually_exclusive_group()
    ginit.add_argument('-r','--reset', action='store_true', help='reset chip')
    ginit.add_argument('-c', '--recalibrate', action='store_true', help='recalibrate FSM')
    ginit.add_argument('-i','--init', action='store_true', help='initialize the chip with default values.')
    ginit.add_argument('-t', '--transmit', action='store_true', help='transmitter on') 
    ginit.add_argument('-x', '--notransmit', action='store_true', help='transmitter off') 
    p.add_argument('-f', '--frequency', type=float, help='set frequency (76.00..108.00)MHz') 
    gstereo = p.add_mutually_exclusive_group()
    gstereo.add_argument('--stereo', action='store_true', help='stereo on') 
    gstereo.add_argument('--mono', action='store_true', help='stereo off') 
    gmute = p.add_mutually_exclusive_group()
    gmute.add_argument('--unmute', action='store_true', help='mute off') 
    gmute.add_argument('--mute', action='store_true', help='mute on') 
    p.add_argument('--pa_target', type=int, help='set pa target (power output)') 
    gapk = p.add_mutually_exclusive_group()
    gapk.add_argument('--clear_apk', action='store_true', help='clear audio peak value') 
    gapk.add_argument('--print_apk', action='store_true', help='print audio peak value in mV') 
    p.add_argument('--rin', type=int, choices=[5,10,20,40], help='set Rin (5,10,20,40)kOhm and print resulting VGA gain in dB') 
    p.add_argument('--input_gain', type=int, choices=[1,2,3,4,5,6], help='set input gain level (1..6) and print resulting VGA gain in dB') 
    p.add_argument('--digital_gain', type=int, choices=[0,1,2], help='set digital gain (0,1,2dB)') 
    p.add_argument('-s', '--status', action='store_true', help='print status and chip id')

    args = p.parse_args()

    q = qn8027()
    if args.reset:
        print('Resetting the chip')
        q.reset_chip()
    elif args.recalibrate:
        print('Triggering recalibration of frequency synthesizer module (FSM)')
        q.start_fsm_recalibration()
    elif args.init:
        print('Initializing chip to default values')
        q.default_init()
    elif args.transmit:
        print('Enabling transmitter')
        q.set_enable_tx(True)
    elif args.notransmit:
        print('Disabling transmitter')
        q.set_enable_tx(False)

    if args.frequency != None:
        if (args.frequency < 76.0) or (args.frequency > 108.0):
            print('ERROR: frequency can only be between 76.0MHz and 108.0MHz')
        else:
            print(f'Setting transmit frequency to {args.frequency}MHz')
            q.set_freq(args.frequency)

    if args.stereo:
        print('Enabling Stereo')
        q.set_enable_stereo(True)
    elif args.mono:
        print('Disabling Stereo')
        q.set_enable_stereo(False)

    if args.mute:
        print('Muting audio')
        q.set_enable_mute(True)
    elif args.unmute:
        print('Unmuting audio')
        q.set_enable_mute(False)

    if args.pa_target:
        q.set_pac(args.pa_target)
        # if transmitting, toggling the transmitter is required after changing the pa target
        if TXREQ(q.txreq.value) == TXREQ.TRANSMIT:
            q.set_enable_tx(False)
            q.set_enable_tx(True)
        q.get_state()
        print(f'Power amplifier target output power set to {q.pa_trgt}dBu (Level={args.pa_target})')

    if args.clear_apk:
        q.reset_aud_pk()
        print('Audio peak level cleared')
    elif args.print_apk:
        print(f'Audio peak level: {q.aud_pk}mV')

    match args.rin:
        case 5:
            q.set_rin(VGA_R_IN.RIN_5KOHM)
        case 10:
            q.set_rin(VGA_R_IN.RIN_10KOHM)
        case 20:
            q.set_rin(VGA_R_IN.RIN_20KOHM)
        case 40:
            q.set_rin(VGA_R_IN.RIN_40KOHM)

    match args.input_gain:
        case 1:
            q.set_gvga(VGA_GVGA_LEVEL.L1)
        case 2:
            q.set_gvga(VGA_GVGA_LEVEL.L2)
        case 3:
            q.set_gvga(VGA_GVGA_LEVEL.L3)
        case 4:
            q.set_gvga(VGA_GVGA_LEVEL.L4)
        case 5:
            q.set_gvga(VGA_GVGA_LEVEL.L5)
        case 6:
            q.set_gvga(VGA_GVGA_LEVEL.L6)

    if args.input_gain != None or args.rin != None:
        q.get_state()
        print(f'VGA gain set to {q.tx_buffer_gain}db with {q.rin.name}, VGA level {q.gvga.name}')

    if args.digital_gain:
        q.set_gdb(VGA_GDB(args.digital_gain))
        print(f'Digital gain set to {args.digital_gain}dB')

    if args.status:
        q.dbg_print_state()

if __name__ == '__main__':
    main()