import argparse
import qn8027

q = qn8027.qn8027()

def print_state(hexvals=False):
    if (hexvals):
        print(f'[REG_SYSTEM] 0x{q.state[qn8027.REG_SYSTEM]:02x} {q.state[qn8027.REG_SYSTEM]:2d} 0b{q.state[qn8027.REG_SYSTEM]:08b}')
    print(f'[REG_SYSTEM] swrst={q.swrst.name} recal={q.recal.name} txreq={q.txreq.name} mono={q.mono.name} mute={q.mute.name} rdsrdy={q.rdsrdy.name}')

    print(f'[FREQ] {str(q.freq)}MHz = 76 + freqhi={str(q.freqhi)} + freqlo={str(q.freqlo)}')
  
    if (hexvals):
        print(f'[REG_GPLT] 0x{q.state[qn8027.REG_GPLT]:02x} {q.state[qn8027.REG_GPLT]:2d} 0b{q.state[qn8027.REG_GPLT]:08b}')
    print(f'[REG_GPLT] tc={q.tc.name} priv_en={q.priv_en.name} t1m_sel={q.t1m_sel.name} gain_txplt={q.gain_txplt.name}')

    if (hexvals):
        print(f'[REG_XTL] 0x{q.state[qn8027.REG_XTL]:02x} {q.state[qn8027.REG_XTL]:2d} 0b{q.state[qn8027.REG_XTL]:08b}')
    print(f'[REG_XTL] xinj={q.xinj.name} xisel={q.xisel}uA')

    if (hexvals):
        print(f'[REG_VGA] 0x{q.state[qn8027.REG_VGA]:02x} {q.state[qn8027.REG_VGA]:2d} 0b{q.state[qn8027.REG_VGA]:08b}')
    print(f'[REG_VGA] xsel={q.xsel.name} tx_buffer_gain={q.tx_buffer_gain}dB gdb={q.gdb.name} rin={q.rin.name} gvga={q.gvga.name}')

    if (hexvals):
        print(f'[REG_CID1] 0x{q.state[qn8027.REG_CID1]:02x} {q.state[qn8027.REG_CID1]:2d} 0b{q.state[qn8027.REG_CID1]:08b}')
        print(f'[REG_CID2] 0x{q.state[qn8027.REG_CID2]:02x} {q.state[qn8027.REG_CID2]:2d} 0b{q.state[qn8027.REG_CID2]:08b}')
    print(f'[CID] familiy={q.cid_product_family} product={q.cid_product} revision={q.cid_major_revision}.{q.cid_minor_revision}')

    if (hexvals):
        print(f'[REG_STATUS] 0x{q.state[qn8027.REG_STATUS]:02x} {q.state[qn8027.REG_STATUS]:2d} 0b{q.state[qn8027.REG_STATUS]:08b}')
    print(f'[REG_STATUS] fsmstate={q.fsmstate.name} rds_upd={q.rds_upd.name} aud_pk={q.aud_pk}mV')

    if (hexvals):
        print(f'[REG_PAC] 0x{q.state[qn8027.REG_PAC]:02x} {q.state[qn8027.REG_PAC]:2d} 0b{q.state[qn8027.REG_PAC]:08b}')
    print(f'[REG_PAC] txpd_clr={q.txpd_clr} pa_trgt={q.pa_trgt}dBu (pa_trgt_reg={q.state[qn8027.REG_PAC] & 0b01111111})')

    print(f'[REG_30_ANTENNA] 0x{q.state[qn8027.REG_30_ANTENNA]}')

    if (hexvals):
        print(f'[REG_FDEV] 0x{q.state[qn8027.REG_FDEV]:02x} {q.state[qn8027.REG_FDEV]:2d} 0b{q.state[qn8027.REG_FDEV]:08b}')
    print(f'[REG_FDEV] tx_fdev={q.tx_fdev}kHz')

    if (hexvals):
        print(f'[REG_RDS] 0x{q.state[qn8027.REG_RDS]:02x} {q.state[qn8027.REG_RDS]:2d} 0b{q.state[qn8027.REG_RDS]:08b}')
    print(f'[REG_RDS] rdsen={q.rdsen.name} rdsfdev={q.rdsfdev}kHz')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('-v','--verbose', action='store_true', help='verbose mode: print hex values of registers')
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
        if qn8027.TXREQ(q.txreq.value) == qn8027.TXREQ.TRANSMIT:
            q.set_enable_tx(False)
            q.set_enable_tx(True)
        q.get_state()
        print(f'Power amplifier target output power set to {q.pa_trgt}dBu (Level={args.pa_target & 0b01111111})')

    if args.clear_apk:
        q.reset_aud_pk()
        print('Audio peak level cleared')
    elif args.print_apk:
        print(f'Audio peak level: {q.aud_pk}mV')

    match args.rin:
        case 5:
            q.set_rin(qn8027.VGA_R_IN.RIN_5KOHM)
        case 10:
            q.set_rin(qn8027.VGA_R_IN.RIN_10KOHM)
        case 20:
            q.set_rin(qn8027.VGA_R_IN.RIN_20KOHM)
        case 40:
            q.set_rin(qn8027.VGA_R_IN.RIN_40KOHM)

    match args.input_gain:
        case 1:
            q.set_gvga(qn8027.VGA_GVGA_LEVEL.L1)
        case 2:
            q.set_gvga(qn8027.VGA_GVGA_LEVEL.L2)
        case 3:
            q.set_gvga(qn8027.VGA_GVGA_LEVEL.L3)
        case 4:
            q.set_gvga(qn8027.VGA_GVGA_LEVEL.L4)
        case 5:
            q.set_gvga(qn8027.VGA_GVGA_LEVEL.L5)
        case 6:
            q.set_gvga(qn8027.VGA_GVGA_LEVEL.L6)

    if args.input_gain != None or args.rin != None:
        q.get_state()
        print(f'VGA gain set to {q.tx_buffer_gain}db with {q.rin.name}, VGA level {q.gvga.name}')

    if args.digital_gain:
        q.set_gdb(qn8027.VGA_GDB(args.digital_gain))
        print(f'Digital gain set to {args.digital_gain}dB')

    if args.status:
        q.get_state()
        print_state(args.verbose)

if __name__ == '__main__':
    main()