#!/usr/bin/env python
"""
setup board.h for linux
"""

import argparse
import sys
import fnmatch
import os
import shlex
import pickle
import re
import shutil
import filecmp

parser = argparse.ArgumentParser(prog='LinuxHWDef',
                                 description='Generate Linux Hardware Definition for ArduPilot')
parser.add_argument(
    '-D', '--outdir', type=str, default=None, help='Output directory')
parser.add_argument(
    '--bootloader', action='store_true', default=False, help='configure for bootloader')
parser.add_argument(
    '--signed-fw', action='store_true', default=False, help='configure for signed FW')
parser.add_argument(
    'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
parser.add_argument(
    '--params', type=str, default=None, help='user default params path')

args = parser.parse_args()

## TODO : add libgpio as standard
## TODO : allow RPI hack ?

# output variables for each pin
f4f7_vtypes = ['MODER', 'OTYPER', 'OSPEEDR', 'PUPDR', 'ODR', 'AFRL', 'AFRH']
f1_vtypes = ['CRL', 'CRH', 'ODR']
f1_input_sigs = ['RX', 'MISO', 'CTS']
f1_output_sigs = ['TX', 'MOSI', 'SCK', 'RTS', 'CH1', 'CH2', 'CH3', 'CH4']
af_labels = ['USART', 'UART', 'SPI', 'I2C', 'SDIO', 'SDMMC', 'OTG', 'JT', 'TIM', 'CAN', 'QUADSPI']

default_gpio = ['INPUT', 'FLOATING']


vtypes = []

# number of pins in each port
pincount = {
    'A': 16,
    'B': 16,
    'C': 16,
    'D': 16,
    'E': 16,
    'F': 16,
    'G': 16,
    'H': 2,
    'I': 0,
    'J': 0,
    'K': 0
}

ports = pincount.keys()

portmap = {}

# dictionary of all config lines, indexed by first word
config = {}

# alternate pin mappings
altmap = {}

# list of all pins in config file order
allpins = []

# list of configs by type
bytype = {}

# list of alt configs by type
alttype = {}

# list of configs by label
bylabel = {}

# list of alt configs by label
altlabel = {}

# list of SPI devices
spidev = []

# list of QSPI devices
qspidev = []

# dictionary of ROMFS files
romfs = {}

# SPI bus list
spi_list = []

# list of QSPI devices
qspi_list = []

# all config lines in order
alllines = []

# allow for extra env vars
env_vars = {}

# build flags for ChibiOS makefiles
build_flags = []

# sensor lists
imu_list = []
compass_list = []
baro_list = []
airspeed_list = []

all_lines = []

dma_exclude_pattern = []

# map from uart names to SERIALn numbers
uart_serial_num = {}

mcu_type = None
dual_USB_enabled = False

# list of device patterns that can't be shared
dma_noshare = []

# integer defines
intdefines = {}


def is_int(str):
    """check if a string is an integer"""
    try:
        int(str)
    except TypeError:
        return False
    return True


def error(errmsg):
    """show an error and exit"""
    print("Error: " + errmsg)
    sys.exit(1)


def have_type_prefix(ptype):
    '''return True if we have a peripheral starting with the given peripheral type'''
    for t in list(bytype.keys()) + list(alttype.keys()):
        if t.startswith(ptype):
            return True
    return False


class generic_pin(object):
    '''class to hold pin definition'''
## TODO : convert of libgpio
    def __init__(self, port, pin, label, type, extra):
        global mcu_series
        self.portpin = "P%s%u" % (port, pin)
        self.port = port
        self.pin = pin
        self.label = label
        self.type = type
        self.extra = extra
        self.af = None
        if type == 'OUTPUT':
            self.sig_dir = 'OUTPUT'
        else:
            self.sig_dir = 'INPUT'
        if mcu_series.startswith("STM32F1") and self.label is not None:
            self.f1_pin_setup()

        # check that labels and pin types are consistent
        for prefix in ['USART', 'UART', 'TIM']:
            if label is None or type is None:
                continue
            if type.startswith(prefix):
                a1 = label.split('_')
                a2 = type.split('_')
                if a1[0] != a2[0]:
                    error("Peripheral prefix mismatch for %s %s %s" % (self.portpin, label, type))


def get_config(name, column=0, required=True, default=None, type=None, spaces=False, aslist=False):
    """get a value from config dictionary"""
    if name not in config:
        if required and default is None:
            error("missing required value %s in hwdef.dat" % name)
        return default
    if aslist:
        return config[name]
    if len(config[name]) < column + 1:
        if not required:
            return None
        error("missing required value %s in hwdef.dat (column %u)" % (name,
                                                                      column))
    if spaces:
        ret = ' '.join(config[name][column:])
    else:
        ret = config[name][column]

    if type is not None:
        if type == int and ret.startswith('0x'):
            try:
                ret = int(ret, 16)
            except TypeError:
                error("Badly formed config value %s (got %s)" % (name, ret))
        else:
            try:
                ret = type(ret)
            except TypeError:
                error("Badly formed config value %s (got %s)" % (name, ret))
    return ret


def load_file_with_include(fname):
    """load a file as an array of lines, processing any include lines"""
    lines = open(fname, 'r').readlines()
    ret = []
    for line in lines:
        if line.startswith("include"):
            a = shlex.split(line)
            if len(a) > 1 and a[0] == "include":
                fname2 = os.path.relpath(os.path.join(os.path.dirname(fname), a[1]))
                ret.extend(load_file_with_include(fname2))
                continue
        ret.append(line)
    return ret


def write_linux_config(f):
    """write linux config defines"""
    f.write('#define LINUX_BOARD_NAME "%s"\n' % os.path.basename(os.path.dirname(args.hwdef[0])))

    f.write('// UART used for stdout (printf)\n')
    if get_config('STDOUT_SERIAL', required=False):
        f.write('#define HAL_STDOUT_SERIAL %s\n\n' % get_config('STDOUT_SERIAL'))
        f.write('// baudrate used for stdout (printf)\n')
        f.write('#define HAL_STDOUT_BAUDRATE %u\n\n' % get_config('STDOUT_BAUDRATE', type=int))

    f.write('\n// APJ board ID (for bootloaders)\n')
    f.write('#define APJ_BOARD_ID %s\n' % get_config('APJ_BOARD_ID'))


    # lib = get_mcu_lib(mcu_type)
    # build_info = lib.build
    #
    # if get_mcu_config('CPU_FLAGS') and get_mcu_config('CORTEX'):
    #     # CPU flags specified in mcu file
    #     cortex = get_mcu_config('CORTEX')
    #     env_vars['CPU_FLAGS'] = get_mcu_config('CPU_FLAGS').split()
    #     build_info['MCU'] = cortex
    #     print("MCU Flags: %s %s" % (cortex, env_vars['CPU_FLAGS']))
    # elif mcu_series.startswith("STM32F1"):
    #     cortex = "cortex-m3"
    #     env_vars['CPU_FLAGS'] = ["-mcpu=%s" % cortex]
    #     build_info['MCU'] = cortex
    # else:
    #     cortex = "cortex-m4"
    #     env_vars['CPU_FLAGS'] = ["-mcpu=%s" % cortex, "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    #     build_info['MCU'] = cortex

    #
    #
    # # setup build variables
    # for v in build_info.keys():
    #     build_flags.append('%s=%s' % (v, build_info[v]))

def write_check_firmware(f):
    '''add AP_CHECK_FIRMWARE_ENABLED if needed'''
    if env_vars.get('AP_PERIPH',0) != 0 or intdefines.get('AP_OPENDRONEID_ENABLED',0) == 1:
        f.write('''
#ifndef AP_CHECK_FIRMWARE_ENABLED
#define AP_CHECK_FIRMWARE_ENABLED 1
#endif
''')

def parse_spi_device(dev):
    '''parse a SPI:xxx device item'''
    a = dev.split(':')
    if len(a) != 2:
        error("Bad SPI device: %s" % dev)
    return 'hal.spi->get_device("%s")' % a[1]


def parse_i2c_device(dev):
    '''parse a I2C:xxx:xxx device item'''
    a = dev.split(':')
    if len(a) != 3:
        error("Bad I2C device: %s" % dev)
    busaddr = int(a[2], base=0)
    if a[1] == 'ALL_EXTERNAL':
        return ('FOREACH_I2C_EXTERNAL(b)', 'GET_I2C_DEVICE(b,0x%02x)' % (busaddr))
    elif a[1] == 'ALL_INTERNAL':
        return ('FOREACH_I2C_INTERNAL(b)', 'GET_I2C_DEVICE(b,0x%02x)' % (busaddr))
    elif a[1] == 'ALL':
        return ('FOREACH_I2C(b)', 'GET_I2C_DEVICE(b,0x%02x)' % (busaddr))
    busnum = int(a[1])
    return ('', 'GET_I2C_DEVICE(%u,0x%02x)' % (busnum, busaddr))


def seen_str(dev):
    '''return string representation of device for checking for duplicates'''
    ret = dev[:2]
    if dev[-1].startswith("BOARD_MATCH("):
        ret.append(dev[-1])
    return str(ret)

def write_IMU_config(f):
    '''write IMU config defines'''
    global imu_list
    devlist = []
    wrapper = ''
    seen = set()
    for dev in imu_list:
        if seen_str(dev) in seen:
            error("Duplicate IMU: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        # get instance number if mentioned
        instance = -1
        aux_devid = -1
        if dev[-1].startswith("INSTANCE:"):
            instance = int(dev[-1][9:])
            dev = dev[:-1]
        if dev[-1].startswith("AUX:"):
            aux_devid = int(dev[-1][4:])
            dev = dev[:-1]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
        n = len(devlist)+1
        devlist.append('HAL_INS_PROBE%u' % n)
        if aux_devid != -1:
            f.write(
            '#define HAL_INS_PROBE%u %s ADD_BACKEND_AUX(AP_InertialSensor_%s::probe(*this,%s),%d)\n'
            % (n, wrapper, driver, ','.join(dev[1:]), aux_devid))
        elif instance != -1:
            f.write(
            '#define HAL_INS_PROBE%u %s ADD_BACKEND_INSTANCE(AP_InertialSensor_%s::probe(*this,%s),%d)\n'
            % (n, wrapper, driver, ','.join(dev[1:]), instance))
        elif dev[-1].startswith("BOARD_MATCH("):
            f.write(
                '#define HAL_INS_PROBE%u %s ADD_BACKEND_BOARD_MATCH(%s, AP_InertialSensor_%s::probe(*this,%s))\n'
                % (n, wrapper, dev[-1], driver, ','.join(dev[1:-1])))
        else:
            f.write(
                '#define HAL_INS_PROBE%u %s ADD_BACKEND(AP_InertialSensor_%s::probe(*this,%s))\n'
                % (n, wrapper, driver, ','.join(dev[1:])))
    if len(devlist) > 0:
        if len(devlist) < 3:
            f.write('#define INS_MAX_INSTANCES %u\n' % len(devlist))
        f.write('#define HAL_INS_PROBE_LIST %s\n\n' % ';'.join(devlist))


def write_MAG_config(f):
    '''write MAG config defines'''
    global compass_list
    devlist = []
    seen = set()
    for dev in compass_list:
        if seen_str(dev) in seen:
            error("Duplicate MAG: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        probe = 'probe'
        wrapper = ''
        a = driver.split(':')
        driver = a[0]
        if len(a) > 1 and a[1].startswith('probe'):
            probe = a[1]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
        n = len(devlist)+1
        devlist.append('HAL_MAG_PROBE%u' % n)
        f.write(
            '#define HAL_MAG_PROBE%u %s ADD_BACKEND(DRIVER_%s, AP_Compass_%s::%s(%s))\n'
            % (n, wrapper, driver, driver, probe, ','.join(dev[1:])))
    if len(devlist) > 0:
        f.write('#define HAL_MAG_PROBE_LIST %s\n\n' % ';'.join(devlist))


def write_BARO_config(f):
    '''write barometer config defines'''
    global baro_list
    devlist = []
    seen = set()
    for dev in baro_list:
        if seen_str(dev) in seen:
            error("Duplicate BARO: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        probe = 'probe'
        wrapper = ''
        a = driver.split(':')
        driver = a[0]
        if len(a) > 1 and a[1].startswith('probe'):
            probe = a[1]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
                if dev[i].startswith('hal.i2c_mgr'):
                    dev[i] = 'std::move(%s)' % dev[i]
        n = len(devlist)+1
        devlist.append('HAL_BARO_PROBE%u' % n)
        args = ['*this'] + dev[1:]
        f.write(
            '#define HAL_BARO_PROBE%u %s ADD_BACKEND(AP_Baro_%s::%s(%s))\n'
            % (n, wrapper, driver, probe, ','.join(args)))
    if len(devlist) > 0:
        f.write('#define HAL_BARO_PROBE_LIST %s\n\n' % ';'.join(devlist))

def write_AIRSPEED_config(f):
    '''write airspeed config defines'''
    global airspeed_list
    devlist = []
    seen = set()
    idx = 0
    for dev in airspeed_list:
        if seen_str(dev) in seen:
            error("Duplicate AIRSPEED: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        wrapper = ''
        a = driver.split(':')
        driver = a[0]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
                if dev[i].startswith('hal.i2c_mgr'):
                    dev[i] = 'std::move(%s)' % dev[i]
        n = len(devlist)+1
        devlist.append('HAL_AIRSPEED_PROBE%u' % n)
        args = ['*this', str(idx)] + dev[1:]
        f.write(
            '#define HAL_AIRSPEED_PROBE%u %s ADD_BACKEND(AP_Airspeed_%s::probe(%s))\n'
            % (n, wrapper, driver, ','.join(args)))
        idx += 1
    if len(devlist) > 0:
        f.write('#define HAL_AIRSPEED_PROBE_LIST %s\n\n' % ';'.join(devlist))
        
def write_board_validate_macro(f):
    '''write board validation macro'''
    global config
    validate_string = ''
    validate_dict = {}
    if 'BOARD_VALIDATE' in config:
        for check in config['BOARD_VALIDATE']:
            check_name = check
            check_string = check
            while True:
                def substitute_alias(m):
                    return '(' + get_config(m.group(1), spaces=True) + ')'
                output = re.sub(r'\$(\w+|\{([^}]*)\})', substitute_alias, check_string)
                if (output == check_string):
                    break
                check_string = output
            validate_dict[check_name] = check_string
        # Finally create check conditional
        for check_name in validate_dict:
            validate_string += "!" + validate_dict[check_name] + "?" + "\"" + check_name + "\"" + ":"
        validate_string += "nullptr"
        f.write('#define HAL_VALIDATE_BOARD (%s)\n\n' % validate_string) 

def get_gpio_bylabel(label):
    '''get GPIO(n) setting on a pin label, or -1'''
    p = bylabel.get(label)
    if p is None:
        return -1
    return p.extra_value('GPIO', type=int, default=-1)


def get_extra_bylabel(label, name, default=None):
    '''get extra setting for a label by name'''
    p = bylabel.get(label)
    if p is None:
        return default
    return p.extra_value(name, type=str, default=default)

def get_UART_ORDER():
    '''get UART_ORDER from SERIAL_ORDER option'''
    if get_config('UART_ORDER', required=False, aslist=True) is not None:
        error('Please convert UART_ORDER to SERIAL_ORDER')
    serial_order = get_config('SERIAL_ORDER', required=False, aslist=True)
    if serial_order is None:
        return None
    if args.bootloader:
        # in bootloader SERIAL_ORDER is treated the same as UART_ORDER
        return serial_order
    map = [ 0, 3, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12 ]
    while len(serial_order) < 4:
        serial_order += ['EMPTY']
    uart_order = []
    global uart_serial_num
    for i in range(len(serial_order)):
        uart_order.append(serial_order[map[i]])
        uart_serial_num[serial_order[i]] = i
    return uart_order

def write_UART_config(f):
    '''write UART config defines'''
    global dual_USB_enabled
    uart_list = get_UART_ORDER()
    if uart_list is None:
        return
    f.write('\n// UART configuration\n')

    # write out driver declarations for HAL_ChibOS_Class.cpp
    devnames = "ABCDEFGHIJ"
    sdev = 0
    idx = 0
    for dev in uart_list:
        if dev == 'EMPTY':
            f.write('#define HAL_UART%s_DRIVER Empty::UARTDriver uart%sDriver\n' %
                    (devnames[idx], devnames[idx]))
            sdev += 1
        else:
            f.write(
                '#define HAL_UART%s_DRIVER ChibiOS::UARTDriver uart%sDriver(%u)\n'
                % (devnames[idx], devnames[idx], sdev))
            sdev += 1
        idx += 1
    for idx in range(len(uart_list), len(devnames)):
        f.write('#define HAL_UART%s_DRIVER Empty::UARTDriver uart%sDriver\n' %
                (devnames[idx], devnames[idx]))

    f.write('#define HAL_WITH_IO_MCU 0\n')
    f.write('\n')

    need_uart_driver = False
    devlist = []
    have_rts_cts = False

    if have_rts_cts:
        f.write('#define AP_FEATURE_RTSCTS 1\n')

    f.write('#define HAL_UART_DEVICE_LIST %s\n\n' % ','.join(devlist))
    if not need_uart_driver and not args.bootloader:
        f.write('''
#ifndef HAL_USE_SERIAL
#define HAL_USE_SERIAL HAL_USE_SERIAL_USB
#endif
''')
    num_uarts = len(devlist)
    if num_uarts > 10:
        error("Exceeded max num UARTs of 10 (%u)" % num_uarts)
    f.write('#define HAL_UART_NUM_SERIAL_PORTS %u\n' % num_uarts)


def write_I2C_config(f):
    '''write I2C config defines'''
    if not have_type_prefix('I2C'):
        print("No I2C peripherals")
        f.write('''
#ifndef HAL_USE_I2C
#define HAL_USE_I2C FALSE
#endif
''')
        return
    if 'I2C_ORDER' not in config:
        error("Missing I2C_ORDER config")
    i2c_list = config['I2C_ORDER']
    f.write('// I2C configuration\n')
    if len(i2c_list) == 0:
        error("I2C_ORDER invalid")
    devlist = []

    f.write('\n#define HAL_I2C_DEVICE_LIST %s\n\n' % ','.join(devlist))

def write_PWM_config(f, ordered_timers):
    pass


def write_ADC_config(f):
    '''write ADC config defines'''
    pass
    # f.write('// ADC config\n')
    # adc_chans = []
    # for l in bylabel:
    #     p = bylabel[l]
    #     if not p.type.startswith('ADC'):
    #         continue
    #     chan = get_ADC1_chan(mcu_type, p.portpin)
    #     scale = p.extra_value('SCALE', default=None)
    #     if p.label == 'VDD_5V_SENS':
    #         f.write('#define ANALOG_VCC_5V_PIN %u\n' % chan)
    #         f.write('#define HAL_HAVE_BOARD_VOLTAGE 1\n')
    #     if p.label == 'FMU_SERVORAIL_VCC_SENS':
    #         f.write('#define FMU_SERVORAIL_ADC_CHAN %u\n' % chan)
    #         f.write('#define HAL_HAVE_SERVO_VOLTAGE 1\n')
    #     adc_chans.append((chan, scale, p.label, p.portpin))
    # adc_chans = sorted(adc_chans)
    # vdd = get_config('STM32_VDD', default='330U')
    # if vdd[-1] == 'U':
    #     vdd = vdd[:-1]
    # vdd = float(vdd) * 0.01
    # f.write('#define HAL_ANALOG_PINS { \\\n')
    # for (chan, scale, label, portpin) in adc_chans:
    #     scale_str = '%.2f/4096' % vdd
    #     if scale is not None and scale != '1':
    #         scale_str = scale + '*' + scale_str
    #     f.write('{ %2u, %12s }, /* %s %s */ \\\n' % (chan, scale_str, portpin,
    #                                                  label))
    # f.write('}\n\n')


def write_GPIO_config(f):
    '''write GPIO config defines'''
    f.write('// GPIO config\n')
    gpios = []
    gpioset = set()
    for l in bylabel:
        p = bylabel[l]
        gpio = p.extra_value('GPIO', type=int)
        if gpio is None:
            continue
        if gpio in gpioset:
            error("Duplicate GPIO value %u" % gpio)
        gpioset.add(gpio)
        # see if it is also a PWM pin
        pwm = p.extra_value('PWM', type=int, default=0)
        port = p.port
        pin = p.pin
        # default config always enabled
        gpios.append((gpio, pwm, port, pin, p, 'true'))
    for alt in altmap.keys():
        for pp in altmap[alt].keys():
            p = altmap[alt][pp]
            gpio = p.extra_value('GPIO', type=int)
            if gpio is None:
                continue
            if gpio in gpioset:
                # check existing entry
                existing_gpio = [item for item in gpios if item[0] == gpio]
                if (existing_gpio[0][4].label == p.label) and (existing_gpio[0][3] == p.pin) and (existing_gpio[0][2] == p.port):
                    # alt item is identical to exiting, do not add again
                    continue
                error("Duplicate GPIO value %u, %s != %s" % (gpio, p, existing_gpio[0][4]))
            pwm = p.extra_value('PWM', type=int, default=0)
            if pwm != 0:
                error("PWM not supported for alt config: %s" % p)
            gpioset.add(gpio)
            port = p.port
            pin = p.pin
            # aux config disabled by defualt
            gpios.append((gpio, pwm, port, pin, p, 'false'))
    gpios = sorted(gpios)
    for (gpio, pwm, port, pin, p, enabled) in gpios:
        f.write('#define HAL_GPIO_LINE_GPIO%u PAL_LINE(GPIO%s,%uU)\n' % (gpio, port, pin))
    f.write('#define HAL_GPIO_PINS { \\\n')
    for (gpio, pwm, port, pin, p, enabled) in gpios:
        f.write('{ %3u, %s, %2u, PAL_LINE(GPIO%s,%uU)}, /* %s */ \\\n' %
                (gpio, enabled, pwm, port, pin, p))
    # and write #defines for use by config code
    f.write('}\n\n')
    f.write('// full pin define list\n')
    last_label = None
    for l in sorted(list(set(bylabel.keys()))):
        p = bylabel[l]
        label = p.label
        label = label.replace('-', '_')
        if label == last_label:
            continue
        last_label = label
        f.write('#define HAL_GPIO_PIN_%-20s PAL_LINE(GPIO%s,%uU)\n' %
                (label, p.port, p.pin))
    f.write('\n')


def write_ROMFS():
    '''create ROMFS embedded header'''
    romfs_list = []
    for k in romfs.keys():
        romfs_list.append((k, romfs[k]))
    env_vars['ROMFS_FILES'] = romfs_list


def write_all_lines(hwdat):
    f = open(hwdat, 'w')
    f.write('\n'.join(all_lines))
    f.close()
    if not 'AP_PERIPH' in env_vars:
        romfs["hwdef.dat"] = hwdat


def write_hwdef_header(outfilename):
    '''write hwdef header file'''
    print("Writing hwdef setup in %s" % outfilename)
    tmpfile = outfilename + ".tmp"
    f = open(tmpfile, 'w')

    f.write('''/*
 generated hardware definitions from hwdef.dat - DO NOT EDIT
*/

#pragma once

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define MHZ (1000U*1000U)
#define KHZ (1000U)

''')

    if args.signed_fw:
        f.write('''
#define AP_SIGNED_FIRMWARE 1
''')
    else:
        f.write('''
#define AP_SIGNED_FIRMWARE 0
''')


    # write_mcu_config(f)
    # write_SPI_config(f)
    # write_QSPI_config(f)
    write_ADC_config(f)
    write_GPIO_config(f)
    write_IMU_config(f)
    write_MAG_config(f)
    write_BARO_config(f)
    write_AIRSPEED_config(f)
    write_board_validate_macro(f)
    write_check_firmware(f)


    # write_PWM_config(f, ordered_timers)
    write_I2C_config(f)
    write_UART_config(f)

    # setup_apj_IDs()
    # write_USB_config(f)


    f.close()
    # see if we ended up with the same file, on an unnecessary reconfigure
    try:
        if filecmp.cmp(outfilename, tmpfile):
            print("No change in hwdef.h")
            os.unlink(tmpfile)
            return
    except Exception:
        pass
    try:
        os.unlink(outfilename)
    except OSError:
        pass
    os.rename(tmpfile, outfilename)


def write_env_py(filename):
    '''write out env.py for environment variables to control the build process'''

    # see if board has a defaults.parm file or a --default-parameters file was specified
    defaults_filename = os.path.join(os.path.dirname(args.hwdef[0]), 'defaults.parm')
    defaults_path = os.path.join(os.path.dirname(args.hwdef[0]), args.params)

    if not args.bootloader:
        if os.path.exists(defaults_path):
            env_vars['DEFAULT_PARAMETERS'] = os.path.abspath(defaults_path)
            print("Default parameters path from command line: %s" % defaults_path)
        elif os.path.exists(defaults_filename):
            env_vars['DEFAULT_PARAMETERS'] = os.path.abspath(defaults_filename)
            print("Default parameters path from hwdef: %s" % defaults_filename)
        else:
            print("No default parameter file found")

    # CHIBIOS_BUILD_FLAGS is passed to the ChibiOS makefile
    env_vars['LINUX_BUILD_FLAGS'] = ' '.join(build_flags)
    pickle.dump(env_vars, open(filename, "wb"))

def romfs_add(romfs_filename, filename):
    '''add a file to ROMFS'''
    romfs[romfs_filename] = filename


def romfs_wildcard(pattern):
    '''add a set of files to ROMFS by wildcard'''
    base_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
    (pattern_dir, pattern) = os.path.split(pattern)
    for f in os.listdir(os.path.join(base_path, pattern_dir)):
        if fnmatch.fnmatch(f, pattern):
            romfs[f] = os.path.join(pattern_dir, f)

def romfs_add_dir(subdirs):
    '''add a filesystem directory to ROMFS'''
    for dirname in subdirs:
        romfs_dir = os.path.join(os.path.dirname(args.hwdef[0]), dirname)
        if not args.bootloader and os.path.exists(romfs_dir):
            for root, d, files in os.walk(romfs_dir):
                for f in files:
                    if fnmatch.fnmatch(f, '*~'):
                        # skip editor backup files
                        continue
                    fullpath = os.path.join(root, f)
                    relpath = os.path.normpath(os.path.join(dirname, os.path.relpath(root, romfs_dir), f))
                    romfs[relpath] = fullpath

def valid_type(ptype, label):
    '''check type of a pin line is valid'''
    patterns = [ 'INPUT', 'OUTPUT', 'TIM\d+', 'USART\d+', 'UART\d+', 'ADC\d+',
                'SPI\d+', 'OTG\d+', 'SWD', 'CAN\d?', 'I2C\d+', 'CS',
                'SDMMC\d+', 'SDIO', 'QUADSPI\d' ]
    matches = False
    for p in patterns:
        if re.match(p, ptype):
            matches = True
            break
    if not matches:
        return False
    # special checks for common errors
    m1 = re.match('TIM(\d+)', ptype)
    m2 = re.match('TIM(\d+)_CH\d+', label)
    if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
        '''timer numbers need to match'''
        return False
    m1 = re.match('CAN(\d+)', ptype)
    m2 = re.match('CAN(\d+)_(RX|TX)', label)
    if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
        '''CAN numbers need to match'''
        return False
    if ptype == 'OUTPUT' and re.match('US?ART\d+_(TXINV|RXINV)', label):
        return True
    m1 = re.match('USART(\d+)', ptype)
    m2 = re.match('USART(\d+)_(RX|TX|CTS|RTS)', label)
    if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
        '''usart numbers need to match'''
        return False
    m1 = re.match('UART(\d+)', ptype)
    m2 = re.match('UART(\d+)_(RX|TX|CTS|RTS)', label)
    if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
        '''uart numbers need to match'''
        return False
    return True

def process_line(line):
    '''process one line of pin definition file'''
    global allpins, imu_list, compass_list, baro_list, airspeed_list
    global mcu_type, mcu_series, default_gpio
    all_lines.append(line)
    a = shlex.split(line, posix=False)
    # keep all config lines for later use
    alllines.append(line)


    config[a[0]] = a[1:]
    if a[0] == 'MCU':
        mcu_type = a[2]
        mcu_series = a[1]
        # setup_mcu_type_defaults()
    elif a[0] == 'SPIDEV':
        spidev.append(a[1:])
    elif a[0] == 'QSPIDEV':
        qspidev.append(a[1:])
    elif a[0] == 'IMU':
        imu_list.append(a[1:])
    elif a[0] == 'COMPASS':
        compass_list.append(a[1:])
    elif a[0] == 'BARO':
        baro_list.append(a[1:])
    elif a[0] == 'AIRSPEED':
        airspeed_list.append(a[1:])
    elif a[0] == 'undef':
        for u in a[1:]:
            print("Removing %s" % u)
            config.pop(u, '')
            bytype.pop(u, '')
            bylabel.pop(u, '')
            alttype.pop(u, '')
            altlabel.pop(u, '')
            for dev in spidev:
                if u == dev[0]:
                    spidev.remove(dev)
            # also remove all occurences of defines in previous lines if any
            for line in alllines[:]:
                if line.startswith('define') and u == line.split()[1]:
                    alllines.remove(line)
            newpins = []
            for pin in allpins:
                if pin.type == u or pin.label == u or pin.portpin == u:
                    if pin.label is not None:
                        bylabel.pop(pin.label, '')
                    portmap[pin.port][pin.pin] = generic_pin(pin.port, pin.pin, None, 'INPUT', [])
                    continue
                newpins.append(pin)
            allpins = newpins
            if u == 'IMU':
                imu_list = []
            if u == 'COMPASS':
                compass_list = []
            if u == 'BARO':
                baro_list = []
            if u == 'AIRSPEED':
                airspeed_list = []
    elif a[0] == 'env':
        print("Adding environment %s" % ' '.join(a[1:]))
        if len(a[1:]) < 2:
            error("Bad env line for %s" % a[0])
        env_vars[a[1]] = ' '.join(a[2:])


def process_file(filename):
    '''process a hwdef.dat file'''
    try:
        f = open(filename, "r")
    except OSError:
        error("Unable to open file %s" % filename)
        return
    for line in f.readlines():
        line = line.split('#')[0] # ensure we discard the comments
        line = line.strip()
        if len(line) == 0 or line[0] == '#':
            continue
        a = shlex.split(line)
        if a[0] == "include" and len(a) > 1:
            include_file = a[1]
            if include_file[0] != '/':
                dir = os.path.dirname(filename)
                include_file = os.path.normpath(
                    os.path.join(dir, include_file))
            print("Including %s" % include_file)
            process_file(include_file)
        else:
            process_line(line)


# process input file
for fname in args.hwdef:
    process_file(fname)

outdir = args.outdir
if outdir is None:
    outdir = '/tmp'

if "MCU" not in config:
    error("Missing MCU type in config")

mcu_type = get_config('MCU', 1)
print("Setup for MCU %s" % mcu_type)

# write out hw.dat for ROMFS
write_all_lines(os.path.join(outdir, "hw.dat"))

# write out hwdef.h
write_hwdef_header(os.path.join(outdir, "hwdef.h"))

# write out ldscript.ld

# romfs_add_dir(['scripts'])
# romfs_add_dir(['param'])

write_ROMFS(outdir)

write_env_py(os.path.join(outdir, "env.py"))
