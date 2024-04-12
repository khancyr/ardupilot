#!/usr/bin/env python
'''
setup board.h for chibios

AP_FLAKE8_CLEAN

'''

import argparse
import sys
import fnmatch
import os
import shlex
import pickle
import re
import shutil
import filecmp


class SITLHWDef(object):

    # output variables for each pin
    af_labels = ['USART', 'UART', 'SPI', 'I2C', 'SDIO', 'SDMMC', 'OTG', 'JT', 'TIM', 'CAN', 'QUADSPI', 'OCTOSPI', 'ETH', 'MCO']

    def __init__(self, quiet=False, bootloader=False, signed_fw=False, outdir=None, hwdef=[], default_params_filepath=None) -> None:
        self.outdir = outdir
        self.hwdef = hwdef
        self.bootloader = bootloader
        self.signed_fw = signed_fw
        self.default_params_filepath = default_params_filepath
        self.quiet = quiet

        # if true then parameters will be appended in special apj-tool
        # section at end of binary:
        self.force_apj_default_parameters = False

        self.default_gpio = ['INPUT', 'FLOATING']

        self.vtypes = []

        # number of pins in each port
        self.pincount = {
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

        self.ports = self.pincount.keys()

        self.portmap = {}

        # dictionary of all config lines, indexed by first word
        self.config = {}

        # alternate pin mappings
        self.altmap = {}

        # list of all pins in config file order
        self.allpins = []

        # list of configs by type
        self.bytype = {}

        # list of alt configs by type
        self.alttype = {}

        # list of configs by label
        self.bylabel = {}

        # list of alt configs by label
        self.altlabel = {}

        # list of SPI devices
        self.spidev = []

        # list of WSPI devices
        self.wspidev = []

        # dictionary of ROMFS files
        self.romfs = {}

        # SPI bus list
        self.spi_list = []

        # list of WSPI devices
        self.wspi_list = []

        # all config lines in order
        self.alllines = []

        # allow for extra env vars
        self.env_vars = {}

        # build flags for ChibiOS makefiles
        self.build_flags = []

        # sensor lists
        self.imu_list = []
        self.compass_list = []
        self.baro_list = []
        self.airspeed_list = []

        # output lines:
        self.all_lines = []

        self.dma_exclude_pattern = []

        self.mcu_type = None
        self.dual_USB_enabled = False

        # list of device patterns that can't be shared
        self.dma_noshare = []

        # integer defines
        self.intdefines = {}

        # list of shared up timers
        self.shared_up = []

    def is_int(self, string) -> bool:
        '''check if a string is an integer'''
        try:
            int(string)
        except Exception:
            return False
        return True

    def error(self, string) -> None:
        '''show an error and exit'''
        print("Error: " + string)
        sys.exit(1)


    def have_type_prefix(self, ptype) -> bool:
        '''return True if we have a peripheral starting with the given peripheral type'''
        for t in list(self.bytype.keys()) + list(self.alttype.keys()):
            if t.startswith(ptype):
                return True
        return False


    def get_config(self, name, column=0, required=True, default=None, type=None, spaces=False, aslist=False):
        '''get a value from config dictionary'''
        if name not in self.config:
            if required and default is None:
                self.error("missing required value %s in hwdef.dat" % name)
            return default
        if aslist:
            return self.config[name]
        if len(self.config[name]) < column + 1:
            if not required:
                return None
            self.error("missing required value %s in hwdef.dat (column %u)" %
                       (name, column))
        if spaces:
            ret = ' '.join(self.config[name][column:])
        else:
            ret = self.config[name][column]

        if type is not None:
            if type == int and ret.startswith('0x'):
                try:
                    ret = int(ret, 16)
                except Exception:
                    self.error("Badly formed config value %s (got %s)" % (name, ret))
            else:
                try:
                    ret = type(ret)
                except Exception:
                    self.error("Badly formed config value %s (got %s)" % (name, ret))
        return ret

    def make_line(self, label):
        '''return a line for a label'''
        if label in self.bylabel:
            p = self.bylabel[label]
            line = 'PAL_LINE(GPIO%s,%uU)' % (p.port, p.pin)
        else:
            line = "0"
        return line

    def enable_can(self, f) -> None:
        '''setup for a CAN enabled board'''
        # TODO
        pass


    def load_file_with_include(self, fname):
        '''load a file as an array of lines, processing any include lines'''
        lines = open(fname, 'r').readlines()
        ret = []
        for line in lines:
            if line.startswith("include"):
                a = shlex.split(line)
                if len(a) > 1 and a[0] == "include":
                    fname2 = os.path.relpath(os.path.join(os.path.dirname(fname), a[1]))
                    ret.extend(self.load_file_with_include(fname2))
                    continue
            ret.append(line)
        return ret

    def get_numeric_board_id(self) -> None:
        '''return a numeric board ID, which may require mapping a string to a
        number via board_list.txt'''
        # TODO
        pass

    def enable_networking(self, f) -> None:
        f.write('''
#ifndef AP_NETWORKING_ENABLED
#define AP_NETWORKING_ENABLED 1
#endif
''')

    def write_mcu_config(self, f) -> None:
        '''write MCU config defines'''
        f.write('#define SITL_BOARD_NAME "%s"\n' % os.path.basename(os.path.dirname(args.hwdef[0])))
        f.write('// SITL type (SITL define)\n')
        f.write('#define %s_SITLCONF\n' % self.get_config('SITL'))
        sitl_subtype = self.get_config('SITL', 1)
        if sitl_subtype[-1:] == 'x' or sitl_subtype[-2:-1] == 'x':
            f.write('#define %s_SITLCONF\n\n' % sitl_subtype[:-2])
        f.write('#define %s\n\n' % sitl_subtype)

        if self.intdefines.get('AP_NETWORKING_ENABLED', 0) == 1:
            self.enable_networking(f)

        if self.have_type_prefix('CAN'):
            self.enable_can(f)

        f.write('\n// CPU serial number (12 bytes)\n')
        # TODO

        f.write('''
#ifndef HAL_ENABLE_THREAD_STATISTICS
#define HAL_ENABLE_THREAD_STATISTICS FALSE
#endif
    ''')

        # setup build variables
        # for v in build_info.keys():
        #     self.build_flags.append('%s=%s' % (v, build_info[v]))

    def write_USB_config(self, f) -> None:
        '''write USB config defines'''
        pass

    def write_check_firmware(self, f) -> None:
        '''add AP_CHECK_FIRMWARE_ENABLED if needed'''
        if self.is_periph_fw() or self.intdefines.get('AP_OPENDRONEID_ENABLED', 0) == 1:
            f.write('''
#ifndef AP_CHECK_FIRMWARE_ENABLED
#define AP_CHECK_FIRMWARE_ENABLED 1
#endif
''')

    def seen_str(self, dev) -> str:
        '''return string representation of device for checking for duplicates'''
        ret = dev[:2]
        if dev[-1].startswith("BOARD_MATCH("):
            ret.append(dev[-1])
        return str(ret)


    def write_board_validate_macro(self, f) -> None:
        '''write board validation macro'''
        validate_string = ''
        validate_dict = {}
        if 'BOARD_VALIDATE' in self.config:
            for check in self.config['BOARD_VALIDATE']:
                check_name = check
                check_string = check
                while True:
                    def substitute_alias(m):
                        return '(' + self.get_config(m.group(1), spaces=True) + ')'
                    output = re.sub(r'\$(\w+|\{([^}]*)\})', substitute_alias, check_string)
                    if (output == check_string):
                        break
                    check_string = output
                validate_dict[check_name] = check_string
            # Finally create check conditional
            for check_name in sorted(validate_dict.keys()):
                validate_string += "!" + validate_dict[check_name] + "?" + "\"" + check_name + "\"" + ":"
            validate_string += "nullptr"
            f.write('#define HAL_VALIDATE_BOARD (%s)\n\n' % validate_string)

    def get_extra_bylabel(self, label, name, default=None):
        '''get extra setting for a label by name'''
        p = self.bylabel.get(label)
        if p is None:
            return default
        return p.extra_value(name, type=str, default=default)

    def write_UART_config(self, f) -> None:
        '''write UART config defines'''
        serial_list = self.get_config('SERIAL_ORDER', required=False, aslist=True)
        if serial_list is None:
            return
        while len(serial_list) < 3: # enough ports for CrashCatcher UART discovery
            serial_list += ['EMPTY']
        f.write('\n// UART configuration\n')

        # write out which serial ports we actually have
        nports = 0
        for idx, serial in enumerate(serial_list):
            if serial == 'EMPTY':
                f.write('#define HAL_HAVE_SERIAL%u 0\n' % idx)
            else:
                f.write('#define HAL_HAVE_SERIAL%u 1\n' % idx)
                nports = nports + 1
        f.write('#define HAL_NUM_SERIAL_PORTS %u\n' % nports)


        need_uart_driver = False
        OTG2_index = None
        devlist = []
        have_rts_cts = False

        for num, dev in enumerate(serial_list):
            if dev.startswith('UART'):
                n = int(dev[4:])
            elif dev.startswith('USART'):
                n = int(dev[5:])
            elif dev.startswith('OTG'):
                n = int(dev[3:])
            elif dev.startswith('EMPTY'):
                devlist.append('{}')
                continue
            else:
                self.error("Invalid element %s in SERIAL_ORDER" % dev)
            devlist.append('HAL_%s_CONFIG' % dev)

        num_ports = len(devlist)
        if 'IOMCU_UART' in self.config:
            num_ports -= 1
        if num_ports > 10:
            self.error("Exceeded max num SERIALs of 10 (%u)" % num_ports)
        f.write('#define HAL_UART_NUM_SERIAL_PORTS %u\n' % num_ports)

    def setup_apj_IDs(self) -> None:
        '''setup the APJ board IDs'''
        self.env_vars['APJ_BOARD_ID'] = self.get_numeric_board_id()
        self.env_vars['APJ_BOARD_TYPE'] = self.get_config('APJ_BOARD_TYPE', default=self.mcu_type)

    def write_all_lines(self, hwdat) -> None:
        f = open(hwdat, 'w')
        f.write('\n'.join(self.all_lines))
        f.close()
        if not self.is_periph_fw():
            self.romfs["hwdef.dat"] = hwdat

    def write_defaulting_define(self, f, name, value) -> None:
        f.write(f"#ifndef {name}\n")
        f.write(f"#define {name} {value}\n")
        f.write("#endif\n")

    def write_define(self, f, name, value) -> None:
        f.write(f"#define {name} {value}\n")

    def write_hwdef_header(self, outfilename) -> None:
        '''write hwdef header file'''
        self.progress("Writing hwdef setup in %s" % outfilename)
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

        self.write_mcu_config(f)
        # self.write_SPI_config(f)
        # self.write_WSPI_config(f)
        # self.write_ADC_config(f)
        # self.write_GPIO_config(f)
        # self.write_IMU_config(f)
        # self.write_MAG_config(f)
        # self.write_BARO_config(f)
        # self.write_AIRSPEED_config(f)
        # self.write_board_validate_macro(f)
        # self.write_check_firmware(f)

        # self.write_peripheral_enable(f)

        if os.path.exists(self.processed_defaults_filepath()):
            self.write_define(f, 'AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED', 1)
        else:
            self.write_define(f, 'AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED', 0)

        if not self.is_bootloader_fw():
            self.write_UART_config(f)
        # else:
        #     self.write_UART_config_bootloader(f)

        # self.setup_apj_IDs()
        # self.write_USB_config(f)

        self.add_normal_firmware_defaults(f)

        f.close()
        # see if we ended up with the same file, on an unnecessary reconfigure
        try:
            if filecmp.cmp(outfilename, tmpfile):
                self.progress("No change in hwdef.h")
                os.unlink(tmpfile)
                return
        except Exception:
            pass
        try:
            os.unlink(outfilename)
        except Exception:
            pass
        os.rename(tmpfile, outfilename)

    def get_processed_defaults_file(self, defaults_filepath, depth=0):
        '''reads defaults_filepath, expanding any @include lines to include
        the contents of the so-references file - recursively.'''
        if depth > 10:
            raise Exception("include loop")
        ret = ""
        with open(defaults_filepath, 'r') as defaults_fh:
            while True:
                line = defaults_fh.readline()
                if line == "":
                    break
                m = re.match(r"^@include\s*([^\s]+)", line)
                if m is None:
                    ret += line
                    continue
                # we've found an include; do that...
                include_filepath = os.path.join(os.path.dirname(defaults_filepath), m.group(1))
                try:
                    # ret += "# Begin included file (%s)" % include_filepath
                    ret += self.get_processed_defaults_file(include_filepath, depth=depth+1)
#                    ret += "# End included file (%s)" % include_filepath
                except FileNotFoundError:
                    raise Exception("%s includes %s but that filepath was not found" %
                                    (defaults_filepath, include_filepath))
        return ret

    def write_processed_defaults_file(self, filepath) -> bool:
        # see if board has a defaults.parm file or a --default-parameters file was specified
        defaults_filename = os.path.join(os.path.dirname(args.hwdef[0]), 'defaults.parm')
        if args.params:
            defaults_path = os.path.join(os.path.dirname(args.hwdef[0]), args.params)
        else:
            defaults_path = ""

        defaults_abspath = None
        if os.path.exists(defaults_path):
            defaults_abspath = os.path.abspath(self.default_params_filepath)
            self.progress("Default parameters path from command line: %s" % self.default_params_filepath)
        elif os.path.exists(defaults_filename):
            defaults_abspath = os.path.abspath(defaults_filename)
            self.progress("Default parameters path from hwdef: %s" % defaults_filename)

        if defaults_abspath is None:
            self.progress("No default parameter file found")
            return False

        content = self.get_processed_defaults_file(defaults_abspath)

        with open(filepath, "w") as processed_defaults_fh:
            processed_defaults_fh.write(content)

        return True

    def write_env_py(self, filename):
        '''write out env.py for environment variables to control the build process'''
        # CHIBIOS_BUILD_FLAGS is passed to the ChibiOS makefile
        self.env_vars['SITL_BUILD_FLAGS'] = ' '.join(self.build_flags)
        pickle.dump(self.env_vars, open(filename, "wb"))

    def romfs_add(self, romfs_filename, filename):
        '''add a file to ROMFS'''
        self.romfs[romfs_filename] = filename

    def romfs_wildcard(self, pattern):
        '''add a set of files to ROMFS by wildcard'''
        base_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
        (pattern_dir, pattern) = os.path.split(pattern)
        for f in os.listdir(os.path.join(base_path, pattern_dir)):
            if fnmatch.fnmatch(f, pattern):
                self.romfs[f] = os.path.join(pattern_dir, f)

    def romfs_add_dir(self, subdirs, relative_to_base=False):
        '''add a filesystem directory to ROMFS'''
        for dirname in subdirs:
            if relative_to_base:
                romfs_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', dirname)
            else:
                romfs_dir = os.path.join(os.path.dirname(args.hwdef[0]), dirname)
            if not self.is_bootloader_fw() and os.path.exists(romfs_dir):
                for root, d, files in os.walk(romfs_dir):
                    for f in files:
                        if fnmatch.fnmatch(f, '*~'):
                            # skip editor backup files
                            continue
                        fullpath = os.path.join(root, f)
                        relpath = os.path.normpath(os.path.join(dirname, os.path.relpath(root, romfs_dir), f))
                        if relative_to_base:
                            relpath = relpath[len(dirname)+1:]
                        self.romfs[relpath] = fullpath

    def valid_type(self, ptype, label):
        '''check type of a pin line is valid'''
        patterns = [
            r'INPUT', r'OUTPUT', r'TIM\d+', r'USART\d+', r'UART\d+', r'ADC\d+',
            r'SPI\d+', r'OTG\d+', r'SWD', r'CAN\d?', r'I2C\d+', r'CS',
            r'SDMMC\d+', r'SDIO', r'QUADSPI\d', r'OCTOSPI\d', r'ETH\d', r'RCC',
        ]
        matches = False
        for p in patterns:
            if re.match(p, ptype):
                matches = True
                break
        if not matches:
            return False
        # special checks for common errors
        m1 = re.match(r'TIM(\d+)', ptype)
        m2 = re.match(r'TIM(\d+)_CH\d+', label)
        if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
            '''timer numbers need to match'''
            return False
        m1 = re.match(r'CAN(\d+)', ptype)
        m2 = re.match(r'CAN(\d+)_(RX|TX)', label)
        if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
            '''CAN numbers need to match'''
            return False
        if ptype == 'OUTPUT' and re.match(r'US?ART\d+_(TXINV|RXINV)', label):
            return True
        m1 = re.match(r'USART(\d+)', ptype)
        m2 = re.match(r'USART(\d+)_(RX|TX|CTS|RTS)', label)
        if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
            '''usart numbers need to match'''
            return False
        m1 = re.match(r'UART(\d+)', ptype)
        m2 = re.match(r'UART(\d+)_(RX|TX|CTS|RTS)', label)
        if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
            '''uart numbers need to match'''
            return False
        return True

    def process_line(self, line, depth):
        '''process one line of pin definition file'''
        self.all_lines.append(line)
        a = shlex.split(line, posix=False)
        # keep all config lines for later use
        self.alllines.append(line)

        self.config[a[0]] = a[1:]

        if a[0] == 'SPIDEV':
            self.spidev.append(a[1:])
        elif a[0] == 'QSPIDEV':
            self.wspidev.append(a[1:])
        elif a[0] == 'OSPIDEV':
            self.wspidev.append(a[1:])
        elif a[0] == 'IMU':
            self.imu_list.append(a[1:])
        elif a[0] == 'COMPASS':
            self.compass_list.append(a[1:])
        elif a[0] == 'BARO':
            self.baro_list.append(a[1:])
        elif a[0] == 'AIRSPEED':
            self.airspeed_list.append(a[1:])
        elif a[0] == 'ROMFS':
            self.romfs_add(a[1], a[2])
        elif a[0] == 'ROMFS_WILDCARD':
            self.romfs_wildcard(a[1])
        elif a[0] == 'ROMFS_DIRECTORY':
            self.romfs_add_dir([a[1]], relative_to_base=True)
        elif a[0] == 'undef':
            for u in a[1:]:
                self.progress("Removing %s" % u)
                self.config.pop(u, '')
                self.bytype.pop(u, '')
                self.bylabel.pop(u, '')
                self.alttype.pop(u, '')
                self.altlabel.pop(u, '')
                self.intdefines.pop(u, '')
                for dev in self.spidev:
                    if u == dev[0]:
                        self.spidev.remove(dev)
                # also remove all occurences of defines in previous lines if any
                for line in self.alllines[:]:
                    if line.startswith('define') and u == line.split()[1] or line.startswith('STM32_') and u == line.split()[0]:  # noqa
                        self.alllines.remove(line)
                newpins = []
                for pin in self.allpins:
                    if pin.type == u or pin.label == u or pin.portpin == u:
                        if pin.label is not None:
                            self.bylabel.pop(pin.label, '')
                        self.portmap[pin.port][pin.pin] = self.generic_pin(pin.port, pin.pin, None, 'INPUT', [], self.mcu_type, self.mcu_series, self.get_ADC1_chan, self.get_ADC2_chan, self.get_ADC3_chan, self.af_labels)  # noqa
                        continue
                    newpins.append(pin)
                self.allpins = newpins
                if u == 'IMU':
                    self.imu_list = []
                if u == 'COMPASS':
                    self.compass_list = []
                if u == 'BARO':
                    self.baro_list = []
                if u == 'AIRSPEED':
                    self.airspeed_list = []
                if u == 'ROMFS':
                    self.romfs = {}
        elif a[0] == 'env':
            self.progress("Adding environment %s" % ' '.join(a[1:]))
            if len(a[1:]) < 2:
                self.error("Bad env line for %s" % a[0])
            name = a[1]
            value = ' '.join(a[2:])
            if name == 'AP_PERIPH' and value != "1":
                raise ValueError("AP_PERIPH may only have value 1")
            self.env_vars[name] = value
        elif a[0] == 'define':
            # extract numerical defines for processing by other parts of the script
            result = re.match(r'define\s*([A-Z_0-9]+)\s+([0-9]+)', line)
            if result:
                (name, intvalue) = (result.group(1), int(result.group(2)))
                if name in self.intdefines and self.intdefines[name] == intvalue:
                    msg = f"{name} already in defines with same value"
                    if depth == 0:
                        print(msg)
                        # raise ValueError(msg)

                self.intdefines[name] = intvalue

    def progress(self, message):
        if self.quiet:
            return
        print(message)

    def process_file(self, filename, depth=0):
        '''process a hwdef.dat file'''
        try:
            f = open(filename, "r")
        except Exception:
            self.error("Unable to open file %s" % filename)
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
                self.progress("Including %s" % include_file)
                self.process_file(include_file, depth+1)
            else:
                self.process_line(line, depth)

    def is_bootloader_fw(self):
        return self.bootloader


    def add_firmware_defaults_from_file(self, f, filename, description):
        self.progress("Setting up as %s" % description)

        dirpath = os.path.dirname(os.path.realpath(__file__))
        filepath = os.path.join(dirpath, filename)

        content = open(filepath, 'r').read()
        f.write('''
// %s defaults

%s

// end %s defaults
''' % (description, content, description))

    def is_io_fw(self):
        return int(self.env_vars.get('IOMCU_FW', 0)) != 0

    def add_iomcu_firmware_defaults(self, f):
        '''add default defines IO firmwares'''
        if not self.is_io_fw():
            # not IOMCU firmware
            return

        self.add_firmware_defaults_from_file(f, "defaults_iofirmware.h", "IOMCU Firmware")

    def is_periph_fw(self):
        return int(self.env_vars.get('AP_PERIPH', 0)) != 0

    def is_normal_fw(self):
        if self.is_io_fw():
            # IOMCU firmware
            return False
        if self.is_periph_fw():
            # Periph firmware
            return False
        if self.is_bootloader_fw():
            # guess
            return False
        return True

    def add_normal_firmware_defaults(self, f):
        '''add default defines to builds with are not bootloader, periph or IOMCU'''
        if not self.is_normal_fw():
            return

        self.add_firmware_defaults_from_file(f, "defaults_normal.h", "normal")

    def processed_defaults_filepath(self):
        return os.path.join(self.outdir, "processed_defaults.parm")

    def write_default_parameters(self):
        '''handle default parameters'''

        if self.is_bootloader_fw():
            return

        if self.is_io_fw():
            return

        filepath = self.processed_defaults_filepath()
        if not self.write_processed_defaults_file(filepath):
            return

        if self.get_config('FORCE_APJ_DEFAULT_PARAMETERS', default=False):
            # set env variable so that post-processing in waf uses
            # apj-tool to append parameters to image:
            if os.path.exists(filepath):
                self.env_vars['DEFAULT_PARAMETERS'] = filepath
            return

        self.romfs_add('defaults.parm', filepath)

    def run(self):

        # process input file
        for fname in self.hwdef:
            self.process_file(fname)

        if "SITL" not in self.config:
            self.error("Missing MCU type in config")

        # self.mcu_type = self.get_config('MCU', 1)
        self.progress("Setup for SITL %s" % self.mcu_type)

        # build a list for peripherals for DMA resolver
        # self.periph_list = self.build_peripheral_list()

        # write out a default parameters file, decide how to use it:
        self.write_default_parameters()

        # write out hw.dat for ROMFS
        self.write_all_lines(os.path.join(self.outdir, "hw.dat"))

        # Add ROMFS directories
        self.romfs_add_dir(['scripts'])
        self.romfs_add_dir(['param'])

        # write out hwdef.h
        self.write_hwdef_header(os.path.join(self.outdir, "hwdef.h"))

        # write out ldscript.ld
        # self.write_ldscript(os.path.join(self.outdir, "ldscript.ld"))

        # self.write_ROMFS(self.outdir)

        # copy the shared linker script into the build directory; it must
        # exist in the same directory as the ldscript.ld file we generate.
        # self.copy_common_linkerscript(self.outdir)

        self.write_env_py(os.path.join(self.outdir, "env.py"))


if __name__ == '__main__':

    parser = argparse.ArgumentParser("sitl_hwdef.py")
    parser.add_argument(
        '-D', '--outdir', type=str, default="/tmp", help='Output directory')
    parser.add_argument(
        '--bootloader', action='store_true', default=False, help='configure for bootloader')
    parser.add_argument(
        '--signed-fw', action='store_true', default=False, help='configure for signed FW')
    parser.add_argument(
        'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
    parser.add_argument(
        '--params', type=str, default=None, help='user default params path')
    parser.add_argument(
        '--quiet', action='store_true', default=False, help='quiet running')

    args = parser.parse_args()

    c = SITLHWDef(
        outdir=args.outdir,
        bootloader=args.bootloader,
        signed_fw=args.signed_fw,
        hwdef=args.hwdef,
        default_params_filepath=args.params,
        quiet=args.quiet,
    )
    c.run()
