import serial, time, sys, re
from xmodem import XMODEM
import serials_find
import ReadLine

SCRIPT_DEBUG = 0


class PassthroughEnabled(Exception):
    pass

class PassthroughFailed(Exception):
    pass


def dbg_print(line=''):
    sys.stdout.write(line + '\n')
    sys.stdout.flush()


def bf_passthrough_init(port, requestedBaudrate, half_duplex=False):
    debug = SCRIPT_DEBUG

    sys.stdout.flush()
    dbg_print("======== PASSTHROUGH INIT ========")
    dbg_print("  Trying to initialize %s @ %s" % (port, requestedBaudrate))

    s = serial.Serial(port=port, baudrate=115200,
        bytesize=8, parity='N', stopbits=1,
        timeout=1, xonxoff=0, rtscts=0)
    s.reset_input_buffer()

    rl = ReadLine.ReadLine(s, 3., ['# ', 'CCC'])

    # Send start command '#'
    cnt = s.write(rl.encode("#\n"))
    s.flush()
    if half_duplex:
        s.read(cnt)
    start = rl.read_line().strip()
    #dbg_print("BF INIT: '%s'" % start.replace("\r", ""))
    if not start or not start.endswith("#"):
        raise PassthroughEnabled("No CLI available. Already in passthrough mode?")
    elif "CCC" in start:
        raise PassthroughEnabled("Passthrough already enabled and bootloader active")

    SerialRXindex = ""

    dbg_print("\nAttempting to detect FC UART configuration...")

    s.timeout = 2
    s.reset_input_buffer()
    cnt = s.write(rl.encode("serial\r\n"))
    s.flush()
    if half_duplex:
        s.read(cnt)

    rl.set_delimiters(["\n", "CCC"])
    rl.clear()

    while True:
        line = rl.read_line(2).strip()
        #print("FC: '%s'" % line)
        if not line or "#" in line:
            break

        if line.startswith("serial"):
            if debug:
                dbg_print("  '%s'" % line)
            config = re.search('serial ([0-9]+) ([0-9]+) ', line)
            if config and config.group(2) == "64":
                dbg_print("    ** Serial RX config detected: '%s'" % line)
                SerialRXindex = config.group(1)
                if not debug:
                    break

    if not SerialRXindex:
        raise PassthroughFailed("!!! RX Serial not found !!!!\n  Check configuration and try again...")

    cmd = "serialpassthrough %s %s" % (SerialRXindex, requestedBaudrate, )

    dbg_print("Enabling serial passthrough...")
    dbg_print("  CMD: '%s'" % cmd)
    s.write(rl.encode(cmd + '\n'))
    s.flush()
    time.sleep(.2)
    s.close()

    dbg_print("======== PASSTHROUGH DONE ========")


if __name__ == '__main__':
    try:
        requestedBaudrate = int(sys.argv[1])
    except:
        requestedBaudrate = 420000
    port = serials_find.get_serial_port()
    bf_passthrough_init(port, requestedBaudrate)
