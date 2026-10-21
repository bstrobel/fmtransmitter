import argparse
import qn8027

q = qn8027.qn8027()

def print_state(s: qn8027.qn8027state):
    print(f'[REG_SYSTEM] swrst={s.swrst.name} recal={s.recal.name} txreq={s.txreq.name} mono={s.mono.name} mute={s.mute.name} rdsrdy={s.rdsrdy.name}')
    print(f'[FREQ] {str(s.freq)}MHz')
    print(f'[REG_GPLT] tc={s.tc.name} priv_en={s.priv_en.name} t1m_sel={s.t1m_sel.name} gain_txplt={s.gain_txplt.name}')
    print(f'[REG_XTL] xinj={s.xinj.name} xisel={s.xisel}uA')
    print(f'[REG_VGA] xsel={s.xsel.name} tx_buffer_gain={s.tx_buffer_gain}dB gdb={s.gdb.name} rin={s.rin.name} gvga={s.gvga.name}')
    print(f'[CID] familiy={s.cid_product_family} product={s.cid_product} revision={s.cid_major_revision}.{s.cid_minor_revision}')
    print(f'[REG_STATUS] fsmstate={s.fsmstate.name} rds_upd={s.rds_upd.name} aud_pk={s.aud_pk}mV')
    print(f'[REG_PAC] txpd_clr={s.txpd_clr} pa_trgt={s.pa_trgt}dBu')
    print(f'[REG_FDEV] tx_fdev={s.tx_fdev}kHz')
    print(f'[REG_RDS] rdsen={s.rdsen.name} rdsfdev={s.rdsfdev}kHz')


def main():
    p = argparse.ArgumentParser()
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
        s = q.get_state()
        print(f'Power amplifier target output power set to {s.pa_trgt}dBu (Level={args.pa_target & 0b01111111})')

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
        s = q.get_state()
        print(f'VGA gain set to {s.tx_buffer_gain}db with {s.rin.name}, VGA level {s.gvga.name}')

    if args.digital_gain:
        q.set_gdb(qn8027.VGA_GDB(args.digital_gain))
        print(f'Digital gain set to {args.digital_gain}dB')

    if args.status:
        print_state(q.get_state())

if __name__ == '__main__':
    main()