#!/usr/bin/env python

"""
Runs tests with gcov coverage support.

 AP_FLAKE8_CLEAN
"""
import argparse
import os
import shutil
import subprocess
import sys

os.environ['PYTHONUNBUFFERED'] = '1'

tools_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.realpath(os.path.join(tools_dir, '../..'))


class CoverageRunner(object):
    """Coverage Runner Class."""

    def __init__(self):
        """Set the files Path."""
        self.REPORT_DIR = os.path.join(root_dir, "reports/lcov-report")
        self.INFO_FILE = os.path.join(root_dir, self.REPORT_DIR, "lcov.info")
        self.INFO_FILE_BASE = os.path.join(root_dir, self.REPORT_DIR, "lcov_base.info")
        self.LCOV_LOG = os.path.join(root_dir, "GCOV_lcov.log")
        self.GENHTML_LOG = os.path.join(root_dir, "GCOV_genhtml.log")
        self.CI = os.environ.get("CI", False)

    def progress(self, string):
        """Pretty printer."""
        print("****** %s" % (string,))

    def init_coverage(self):
        """Initialize ArduPilot for coverage.

        This need to be run with the binaries built.
        """
        self.progress("Initializing Coverage...")
        self.progress("Removing previous reports")
        try:
            shutil.rmtree(self.REPORT_DIR)
        except FileNotFoundError:
            pass

        try:
            os.makedirs(self.REPORT_DIR)
        except FileExistsError:
            pass

        self.progress("Checking that vehicles binaries are set up and built")
        examples_dir = os.path.join(root_dir, 'build/linux/examples')
        binaries_dir = os.path.join(root_dir, 'build/sitl/bin')
        if not (self.check_build("example", examples_dir) and self.check_build("binaries", binaries_dir)):
            self.run_build()

        self.progress("Zeroing previous build")
        retcode = subprocess.call(["lcov", "--zerocounters", "--directory", root_dir])
        if retcode != 0:
            self.progress("Failed with retcode (%s)" % retcode)
            exit(1)

        self.progress("Initializing Coverage with current build")
        try:
            result = subprocess.run(["lcov",
                                     "--no-external",
                                     "--initial",
                                     "--capture",
                                     "--exclude", root_dir + "/modules/uavcan/*",
                                     "--exclude", root_dir + "/build/sitl/modules/*",
                                     "--directory", root_dir,
                                     "-o", self.INFO_FILE_BASE,
                                     ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=True)
            if not self.CI:
                print(*result.args)
                print(result.stdout)
            with open(self.LCOV_LOG, 'w') as log_file:
                log_file.write(result.stdout)
        except subprocess.CalledProcessError as err:
            print("ERROR :")
            print(err.cmd)
            print(err.output)
            exit(1)
        self.progress("Initialization done !")

    def check_build(self, name, path):
        """Check that build directory is not empty and that binaries are built with the coverage flags."""
        self.progress("Checking that %s are set up and built" % name)
        if os.path.exists(path):
            if not os.listdir(path):
                return False
        else:
            return False
        self.progress("Checking %s build configuration for coverage" % name)
        with open(os.path.join(path, "../compile_commands.json"), "r") as searchfile:
            for line in searchfile:
                if "-ftest-coverage" in line:
                    return True
            self.progress("%s was't built with coverage support" % name)
            return False

    def run_build(self):
        """Clean the build directory and build binaries for coverage."""
        os.environ["CCFLAGS"] = os.environ.get("CCFLAGS", "") + " -fprofile-arcs -ftest-coverage"
        os.environ["CXXFLAGS"] = os.environ.get("CXXFLAGS", "") + " -fprofile-arcs -ftest-coverage"
        os.environ["LINKFLAGS"] = os.environ.get("LINKFLAGS", "") + " -lgcov -coverage"
        os.environ["COVERAGE"] = "1"
        self.progress("Removing previous build binaries")
        try:
            shutil.rmtree(os.path.join(root_dir, "build"))
        except FileNotFoundError:
            pass

        self.progress("Building examples and SITL binaries")
        os.chdir(root_dir)
        waf_light = os.path.join(root_dir, "modules/waf/waf-light")
        try:
            subprocess.run([waf_light, "configure", "--board=linux", "--debug"], check=True)
            subprocess.run([waf_light, "examples"], check=True)
            subprocess.run([waf_light, "configure", "--debug"], check=True)
            subprocess.run([waf_light], check=True)
        except subprocess.CalledProcessError as err:
            print("ERROR :")
            print(err.cmd)
            print(err.output)
            exit(1)
        self.progress("Build examples and vehicle binaries done !")

    def run_full(self):
        """Run full coverage on maximum of ArduPilot binaries and test functions."""
        self.progress("Running full test suite...")
        self.run_build()
        self.init_coverage()
        self.progress("Running tests")
        SPEEDUP = 5
        TIMEOUT = 14400
        autotest = os.path.join(root_dir, "Tools/autotest/autotest.py")

        self.progress("Running run.examples")
        subprocess.run([autotest,
                        "--timeout=" + str(TIMEOUT),
                        "--debug",
                        "--no-clean",
                        "--speedup=" + str(SPEEDUP),
                        "run.examples"
                        ])
        self.progress("Running run.unit_tests")
        subprocess.run(
            [autotest,
             "--timeout=" + str(TIMEOUT),
             "--debug",
             "--no-clean",
             "build.unit_tests",
             "run.unit_tests"])
        test_list = ["Plane", "QuadPlane", "Sub", "Copter", "Helicopter", "Rover", "Tracker"]
        for test in test_list:
            self.progress("Running test.%s" % test)
            subprocess.run([autotest,
                            "--timeout=" + str(TIMEOUT),
                            "--debug",
                            "--no-clean",
                            "build.%s" % test,
                            "test.%s" % test,
                            ])

        # TODO add any other execution path/s we can to maximise the actually
        # used code, can we run other tests or things?  Replay, perhaps?
        self.update_stats()

    def update_stats(self):
        """Update Coverage statistics only.

        Assumes that coverage tests have been run.
        """
        self.progress("Generating Coverage statistics")
        with open(self.LCOV_LOG, 'a') as log_file:
            # we cannot use subprocess.PIPE and result.stdout to get the output as it will be too long and trigger
            # BlockingIOError: [Errno 11] write could not complete without blocking
            # thus we ouput to temp file, and print the file line by line...
            with open("tmp_file", 'w+') as tmp_file:
                try:
                    subprocess.run(["lcov",
                                    "--no-external",
                                    "--capture",
                                    "--directory", root_dir,
                                    "-o", self.INFO_FILE,
                                    ], stdout=tmp_file, stderr=subprocess.STDOUT, text=True, check=True)
                    if not self.CI:
                        tmp_file.seek(0)
                        content = tmp_file.read().splitlines()
                        for line in content:
                            print(line)
                            log_file.write(line)
                        tmp_file.seek(0)

                    subprocess.run(["lcov",
                                    "--add-tracefile", self.INFO_FILE_BASE,
                                    "--add-tracefile", self.INFO_FILE,
                                    ], stdout=tmp_file, stderr=subprocess.STDOUT, text=True, check=True)
                    if not self.CI:
                        tmp_file.seek(0)
                        content = tmp_file.read().splitlines()
                        for line in content:
                            # print(line)  # not usefull to print
                            log_file.write(line)
                        tmp_file.seek(0)

                    # remove files we do not intentionally test:
                    subprocess.run(["lcov",
                                    "--remove", self.INFO_FILE,
                                    ".waf*",
                                    root_dir + "/modules/gtest/*",
                                    root_dir + "/modules/uavcan/*",
                                    root_dir + "/build/linux/libraries/*",
                                    root_dir + "/build/sitl/libraries/*",
                                    root_dir + "/build/sitl/modules/*",
                                    "-o", self.INFO_FILE
                                    ], stdout=tmp_file, stderr=subprocess.STDOUT, text=True, check=True)
                    if not self.CI:
                        tmp_file.seek(0)
                        content = tmp_file.read().splitlines()
                        for line in content:
                            # print(line)  # not usefull to print
                            log_file.write(line)
                        tmp_file.seek(0)

                except subprocess.CalledProcessError as err:
                    print("ERROR :")
                    print(err.cmd)
                    print(err.output)
                    os.remove("tmp_file")
                    sys.exit(1)
            os.remove("tmp_file")

        with open(self.GENHTML_LOG, 'w+') as log_file:
            try:
                subprocess.run(["genhtml", self.INFO_FILE,
                                "-o", self.REPORT_DIR,
                                ], stdout=log_file, stderr=subprocess.STDOUT, text=True, check=True)
                if not self.CI:
                    log_file.seek(0)
                    content = log_file.read().splitlines()
                    for line in content:
                        print(line)
            except subprocess.CalledProcessError as err:
                print("ERROR :")
                print(err.cmd)
                print(err.output)
                exit(1)
        self.progress("Coverage successful. Open " + self.REPORT_DIR + "/index.html")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs tests with gcov coverage support.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-i', '--init', action='store_true',
                       help='Initialise ArduPilot for coverage. It should be run after building the binaries.')
    group.add_argument('-f', '--full', action='store_true',
                       help='Run ArduPilot full coverage. This will run all tests and example. It is really long.')
    group.add_argument('-b', '--build', action='store_true',
                       help='Clean the build directory and build binaries for coverage.')
    group.add_argument('-u', '--update', action='store_true',
                       help='Update coverage statistics. To be used after running some tests.')

    args = parser.parse_args()

    runner = CoverageRunner()
    if args.init:
        runner.init_coverage()
        sys.exit(0)
    if args.full:
        runner.run_full()
        sys.exit(0)
    if args.build:
        runner.run_build()
        sys.exit(0)
    if args.update:
        runner.update_stats()
        sys.exit(0)
    parser.print_help()
    sys.exit(0)
