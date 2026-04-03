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
    T1MSEL_58s = 0
    T1MSEL_59s = 1
    T1MSEL_60s = 2 #default
    T1MSEL_INFINITY = 3
REG_GPLT_GAINTXPLT_BIT = 0 # Gain of TX pilot to adjust pilot frequency deviation. Refer to peak frequency deviation of MPX signal when audio input is full scale.
class GPLT_GAINTXPLT(Enum):
    GAINTXPLT_7PCT = 7
    GAINTXPLT_8PCT = 8
    GAINTXPLT_9PCT = 9 # default
    GAINTXPLT_10PCT = 10

REG_XTL=0x03
REG_XTL_XINJ_BIT = 6 # Select the reference clock source
class XTL_XINJ(Enum):
    USE_CRYSTAL = 0 # default - Use crystal on XTAL1/XTAL2
    EXT_CLOCK_ON_XTAL1 = 1 # Inject digital clock from XTAL1
    EXT_SINE_CLOCK_ON_XTAL1 = 2 # Single end sine-wave injection on XTAL1
    EXT_DIFFERENTIAL_SINE_CLOCK = 3 # Differential sine-wave injection on XTAL1/2
REG_XTL_XISEL_BIT = 0 # Crystal oscillator current control. 6.25uA*XISEL[5:0], 0-400uA when use crystal on XTAL1/XTAL2.
REG_XTL_XISEL_CURRENT_100uA = 0b01000 # default

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
VGA_GAIN_MX = [
    [3,-3, -9, -15],
    [6, 0, -6, -9],
    [9, 3, -3, -9],
    [12, 6, 0, -6],
    [15, 9, 3, -3],
    [18, 12, 6, 0]
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
REG_PAC_PATRGT_BIT = 0 # PA output power target is 0.62*PA_TRGT+71dBu. Valid values are 20-75.
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


class qn8027:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.state = bytearray(0x13)
        self.get_state()

    def send_cmd(self,reg,val):
        self.i2c.writeto(I2C_ADDR, bytes([reg,val]))

    def send_org_init(self):
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
        self.get_state()

    def recalibrate_fsm(self):
        self.get_state()
        # flip the rcal bit for 10ms
        self.dbg_print_reg_status(True)
        self.dbg_print_reg_system(True)
        self.send_cmd(REG_SYSTEM, 1 << REG_SYSTEM_RECAL_BIT)
        time.sleep(1)
        self.send_cmd(REG_SYSTEM, self.state[REG_SYSTEM] & ~(1 << REG_SYSTEM_RECAL_BIT))
        self.dbg_print_reg_status(True)
        self.dbg_print_reg_system(True)


    def get_state(self):
        self.i2c.writeto_then_readfrom(I2C_ADDR, bytes([REG_SYSTEM]), self.state)

    def dbg_print_reg_system(self,hexvals=False):
        reg_system_val = self.state[REG_SYSTEM]
        swrst = SWRST((reg_system_val >> REG_SYSTEM_SWRST_BIT) & 0b00000001).name
        recal = RECAL((reg_system_val >> REG_SYSTEM_RECAL_BIT) & 0b00000001).name
        txreq = TXREQ((reg_system_val >> REG_SYSTEM_TXREQ_BIT) & 0b00000001 ).name
        mono = MONO((reg_system_val >> REG_SYSTEM_MONO_BIT) & 0b00000001).name
        mute = MUTE((reg_system_val >> REG_SYSTEM_MUTE_BIT) & 0b00000001).name
        rdsrdy = RDSRDY((reg_system_val >> REG_SYSTEM_RDSRDY_BIT) & 0b00000001).name
        ch = str((reg_system_val & 0b00000011) * 256 * 0.05 + 76)
        if (hexvals):
            print(f'[REG_SYSTEM] 0x{reg_system_val:02x} {reg_system_val:2d} 0b{reg_system_val:08b}')
        print(f'[REG_SYSTEM] swrst={swrst} recal={recal} txreq={txreq} mono={mono} mute={mute} rdsrdy={rdsrdy} chHI={ch}')

    def dbg_print_freq(self,hexvals=False):
        reg_system_val = self.state[REG_SYSTEM]
        reg_ch1_val = self.state[REG_CH1]
        freqhi = (reg_system_val & 0b00000011) * 256 * 0.05
        freqlo = reg_ch1_val  * 0.05
        print(f'[FREQ] {str(freqhi + freqlo + 76)}MHz = 76 + freqhi={str(freqhi)} + freqlo={str(freqlo)}')
    
    def dbg_print_reg_gplt(self,hexvals=False):
        reg_gplt_val = self.state[REG_GPLT]
        tc = TC((reg_gplt_val >> REG_GPLT_TC_BIT) & 0b00000001).name
        priv_en = PRIVEN((reg_gplt_val >> REG_GPLT_PRIVEN_BIT) & 0b00000001).name
        t1m_sel = GPLT_T1MSEL((reg_gplt_val >> REG_GPLT_T1MSEL_BIT) & 0b00000011).name
        gain_txplt_val = (reg_gplt_val & 0b00001111)
        if (GPLT_GAINTXPLT.__contains__(gain_txplt_val)):
            gain_txplt = GPLT_GAINTXPLT(gain_txplt_val).name
        else:
            gain_txplt = f'invalid:{str(gain_txplt_val)}'
        if (hexvals):
            print(f'[REG_GPLT] 0x{reg_gplt_val:02x} {reg_gplt_val:2d} 0b{reg_gplt_val:08b}')
        print(f'[REG_GPLT] tc={tc} priv_en={priv_en} t1m_sel={t1m_sel} gain_txplt={gain_txplt}')

    def dbg_print_reg_xtl(self,hexvals=False):
        reg_xtl_val = self.state[REG_XTL]
        xinj = XTL_XINJ((reg_xtl_val >> REG_XTL_XINJ_BIT) & 0b00000011).name
        xisel = str((reg_xtl_val & 0b0011111) * 6.25)
        if (hexvals):
            print(f'[REG_XTL] 0x{reg_xtl_val:02x} {reg_xtl_val:2d} 0b{reg_xtl_val:08b}')
        print(f'[REG_XTL] xinj={xinj} xisel={xisel}uA')

    def dbg_print_reg_vga(self,hexvals=False):
        reg_vga_val = self.state[REG_VGA]
        xsel = VGA_XSEL((reg_vga_val >> REG_VGA_XSEL_BIT) & 0b00000001).name
        rin_val = reg_vga_val & 0b00000011
        gvga_val = (reg_vga_val >> REG_VGA_GVGA_BIT) &0b00000111
        if gvga_val < 0b00000110:
            tx_buffer_gain = VGA_GAIN_MX[gvga_val][rin_val]
        else:
            tx_buffer_gain = f'reserved (GVGA: {gvga_val:03b})'
        gdb = VGA_GDB(gvga_val).name
        rin = VGA_R_IN(rin_val).name
        if (hexvals):
            print(f'[REG_VGA] 0x{reg_vga_val:02x} {reg_vga_val:2d} 0b{reg_vga_val:08b}')
        print(f'[REG_VGA] xsel={xsel} tx_buffer_gain={tx_buffer_gain}dB gdb={gdb} rin={rin} gvga={VGA_GVGA_LEVEL(gvga_val).name}')

    def dbg_print_cid(self,hexvals=False):
        reg_cid1_val = self.state[REG_CID1]
        reg_cid2_val = self.state[REG_CID2]
        if (reg_cid1_val >> REG_CID1_CID1_BIT) & 0b00000111 == REG_CID1_CID1_FM_FAMILY:
            cid_product_family = 'FM'
        else:
            cid_product_family = 'RESERVED'
        if (reg_cid2_val >> REG_CID2_CID3_BIT) & 0b00001111 == REG_CID2_CID3_QN8027:
            cid_product = 'QN8027'
        else:
            cid_product = 'RESERVED'
        cid_minor_revision = (reg_cid1_val & 0b00000011) + 1
        cid_major_revision = (reg_cid2_val & 0b00001111) + 1
        # if (cid_major_revision > 4):
            # cid_major_revision = 'RESERVED'
        if (hexvals):
            print(f'[REG_CID1] 0x{reg_cid1_val:02x} {reg_cid1_val:2d} 0b{reg_cid1_val:08b}')
            print(f'[REG_CID2] 0x{reg_cid2_val:02x} {reg_cid2_val:2d} 0b{reg_cid2_val:08b}')
        print(f'[CID] familiy={cid_product_family} product={cid_product} revision={cid_major_revision}.{cid_minor_revision}')

    def dbg_print_reg_status(self,hexvals=False):
        reg_status_val = self.state[REG_STATUS]
        fsmstate = FSM(reg_status_val & 0b00000111).name
        rds_upd = RDSUPD((reg_status_val >> REG_STATUS_RDSUPD_BIT) & 0b00000001).name
        aud_pk = str(((reg_status_val >> REG_STATUS_AUDPK_BIT) & 0b00001111) * 45)
        if (hexvals):
            print(f'[REG_STATUS] 0x{reg_status_val:02x} {reg_status_val:2d} 0b{reg_status_val:08b}')
        print(f'[REG_STATUS] fsmstate={fsmstate} rds_upd={rds_upd} aud_pk={aud_pk}mV')

    def dbg_print_reg_pac(self,hexvals=False):
        reg_pac_val = self.state[REG_PAC]
        txpd_clr = (reg_pac_val >> REG_PAC_TXPDCLR_BIT) & 0b00000001
        pa_trgt = 0.62 * (reg_pac_val & 0b01111111) + 71
        if (hexvals):
            print(f'[REG_PAC] 0x{reg_pac_val:02x} {reg_pac_val:2d} 0b{reg_pac_val:08b}')
        print(f'[REG_PAC] txpd_clr={txpd_clr} pa_trgt={pa_trgt}dBu')

    def dbg_print_reg_fdev(self,hexvals=False):
        reg_fdev_val = self.state[REG_FDEV]
        tx_fdev = 0.58 * reg_fdev_val
        if (hexvals):
            print(f'[REG_FDEV] 0x{reg_fdev_val:02x} {reg_fdev_val:2d} 0b{reg_fdev_val:08b}')
        print(f'[REG_FDEV] tx_fdev={tx_fdev}kHz')

    def dbg_print_reg_rds(self,hexvals=False):
        reg_rds_val = self.state[REG_RDS]
        rdsen = RDS((reg_rds_val >> REG_RDS_RDSEN_BIT) & 0b00000001).name
        rdsfdev = 0.35 * (reg_rds_val & 0b01111111)
        if (hexvals):
            print(f'[REG_RDS] 0x{reg_rds_val:02x} {reg_rds_val:2d} 0b{reg_rds_val:08b}')
        print(f'[REG_RDS] rdsen={rdsen} rdsfdev={rdsfdev}kHz')

    def dbg_print_state(self, hexvals=False):
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

    def set_freq(self,freqMHz):
        self.get_state()
        channel = int(math.floor(((freqMHz*1000000-76000000))/50000))
        # print(f'channel={channel:08b}, {channel:2d}')
        fHiBits = (channel >> 8) & 255
        fLoBits = channel & 255
        # print(f'fHiBits={fHiBits:08b}, fLoBits={fLoBits:08b}')
        # print(f'reg_system_before={self.state[REG_SYSTEM]:08b}')
        reg_system_new = (self.state[REG_SYSTEM] & 0b11111100) | fHiBits
        # print(f'reg_system_new={reg_system_new:08b}')
        reg_ch1_new = fLoBits
        self.send_cmd(REG_CH1,reg_ch1_new)
        self.send_cmd(REG_SYSTEM,reg_system_new)
        self.get_state()
        # print(f'reg_system_after={self.state[REG_SYSTEM]:08b}')

    def set_enable_tx(self, enableTX=True):
        self.get_state()
        if enableTX:
            reg_system_new = self.state[REG_SYSTEM] | (1 << REG_SYSTEM_TXREQ_BIT)
        else:
            reg_system_new = self.state[REG_SYSTEM] & ~(1 << REG_SYSTEM_TXREQ_BIT)
        self.send_cmd(REG_SYSTEM,reg_system_new)
        self.get_state()

    def set_enable_stereo(self, stereoOn=True):
        self.get_state()
        if stereoOn:
            reg_system_new = self.state[REG_SYSTEM] & ~(1 << REG_SYSTEM_MONO_BIT)
        else:
            reg_system_new = self.state[REG_SYSTEM] | (1 << REG_SYSTEM_MONO_BIT)
        self.send_cmd(REG_SYSTEM,reg_system_new)
        self.get_state()

    def set_enable_mute(self, muteOn=True):
        self.get_state()
        if muteOn:
            reg_system_new = self.state[REG_SYSTEM] | (1 << REG_SYSTEM_MUTE_BIT)
        else:
            reg_system_new = self.state[REG_SYSTEM] & ~(1 << REG_SYSTEM_MUTE_BIT)
        self.send_cmd(REG_SYSTEM,reg_system_new)
        self.get_state()

def main():
    q = qn8027()
    print('>>>>>')
    # q.dbg_print_reg_system()
    # q.dbg_print_reg_status()
    # q.set_enable_tx()
    # q.dbg_print_reg_system()
    # q.dbg_print_reg_status()
    # q.set_enable_tx(False)
    # q.dbg_print_reg_system()
    # q.dbg_print_reg_status()
    # q.set_freq(97.3)
    # q.dbg_print_freq()
    # print('#### Resetting chip')
    # q.reset_chip()
    q.dbg_print_state()
    # print('++++ Sending original init')
    # q.send_org_init()
    # q.set_freq(91.2)
    # q.dbg_print_freq()

if __name__ == '__main__':
    main()