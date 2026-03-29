import board
import digitalio
import busio

# Try to create an I2C device
i2c = busio.I2C(board.SCL, board.SDA)
print("I2C ok!")

addr = 0x2c
xtl_3e = [ 0x03,0x3e ]  # 0b00111110
                        #   ^^-------> Use crystal on XTAL1/XTAL2
                        #     ^^^^^^-> XTAL oscillator current = 387uA
#vga_36 = [ 0x04,0x36 ] # 12MHz crystal
                        # 0b00110110
                        #   ^--------> 12MHz XTAL
                        #    ^^^-----> GVGA = 0dB
                        #       ^^---> GDB = 1dB
                        #         ^^-> RIN = 20kOhm
vga_a6 = [ 0x04,0xa6 ]  # 24MHz crystal
                        # 0b10100110
                        #   ^--------> 24MHz XTAL
                        #    ^^^-----> GVGA = -3dB
                        #       ^^---> GDB = 1dB
                        #         ^^-> RIN = 20kOhm
system_41 = [ 0x00,0x41 ]   # 0b01000001
                            #    ^------- RECAL: Reset the FSM. After this bit is de-asserted, FSM will go through all the power up and calibration sequence.
                            #         ^^- Highest 2 bits of 10-bit channel index: Channel freq is (76+CH*0.05) MHz
system_01 = [ 0x00,0x01 ]   # 0b00000001
                            #    ^------- RECAL: No Reset
                            #         ^^- Highest 2 bits of 10-bit channel index: Channel freq is (76+CH*0.05) MHz

cmd_18_e4 = [ 0x18, 0xe4 ]
cmd_1b_f0 = [ 0x1b, 0xf0 ]
gplt_b9 = [ 0x02, 0xb9 ]    # 0b10111001
                            #   ^-------- TC = 75uS pre-emphasis
                            #    ^------- PRIV disabled
                            #     ^^----- t1m_sel = infinity (disabled)
                            #       ^^^^- TX pilot gain 9% (default)
ch1_74 = [ 0x01, 0x74 ]
ch1_68 = [ 0x01, 0x68 ]
system_22 = [ 0x00,0x22 ]   # 0b00100010
                            #     ^------ TXREQ ON, enter transmit mode
                            #         ^^- Highest 2 bits of 10-bit channel index: Channel freq is (76+CH*0.05) MHz


system_21 = [ 0x00,0x21 ]   # 0b00100001
                            #     ^------ TXREQ ON, enter transmit mode
                            #         ^^- Highest 2 bits of 10-bit channel index: Channel freq is (76+CH*0.05) MHz


init_cmds = [ xtl_3e, vga_a6, system_41, system_01, cmd_18_e4, cmd_1b_f0, gplt_b9, ch1_68, system_21 ]

for cmd in init_cmds:
    i2c.writeto(addr, bytes(cmd))


print("done!")
