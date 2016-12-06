#!/bin/bash
# For more info about pvs-studio see http://www.viva64.com/en/b/0457/ , http://www.viva64.com/en/b/0457/ , http://www.viva64.com/en/m/0036/

set -e
set -x

SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../")

waf() {
    if [ -x ./waf ]; then
        ./waf "$@"
    else
        ./modules/waf/waf-light "$@"
    fi
}

PVS_CHECK_DISABLE="-dV730,V668,V512"

function command_exists {
    if command -v "$1" >/dev/null 2>&1; then
        return 1
    else
        echo "Require $1 but it's not installed. Let's install it.\n"
        return 0
    fi
}

function use_pvs_studio {
    cd "${ARDUPILOT_ROOT}/build/sitl"
    pvs-studio-analyzer analyze -o "${ARDUPILOT_ROOT}"/build/sitl/ardupilot.log -j4
    plog-converter -a GA:1,2 -t tasklist -o "${ARDUPILOT_ROOT}"/ardupilot.tasks "${ARDUPILOT_ROOT}"/build/sitl/ardupilot.log $PVS_CHECK_DISABLE
}

function install_pvs_studio {
    wget http://files.viva64.com/pvs-studio-6.11.20138.1-amd64.deb
    sudo dpkg -i pvs-studio-6.11.20138.1-amd64.deb
    sudo apt update
    #sudo apt upgrade -f
}

function use_pvs_tool {
    cd "${ARDUPILOT_ROOT}"
    how-to-use-pvs-studio-free -c 2 ./
    waf configure
    waf
}

function install_pvs_tool {
    git clone https://github.com/viva64/how-to-use-pvs-studio-free
    cd how-to-use-pvs-studio-free
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make
    sudo make install
    cd ../../
    sudo rm -rf how-to-use-pvs-studio-free/
    cd "${ARDUPILOT_ROOT}"
}


if command_exists how-to-use-pvs-studio-free ; then
    install_pvs_tool
    use_pvs_tool
else

    use_pvs_tool
fi

if command_exists pvs-studio-analyzer ; then
    install_pvs_studio
    use_pvs_studio
else
    use_pvs_studio
fi
