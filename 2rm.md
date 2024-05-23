
Set SCR_ENABLE 1
use RC 8 to set/unset standby
use RC_override in broadcast to send to both FCUs.

Tools/autotest/sim_vehicle.py -v ArduCopter -f + --slave 1 -I0 --sysid 1 --use-dir=FCU1 --add-param-file=$(pwd)/2rm.parm -A "--serial1=tcp:5761 " --console --map --no-rcin

Tools/autotest/sim_vehicle.py -v ArduCopter --model json:0.0.0.0 --slave 0 -I1 --sysid 2 --use-dir=FCU2 --add-param-file=$(pwd)/2rm.parm -A "--serial1=tcpclient:127.0.0.1:5761 " --no-rebuild -m "--console --source-system 252" --no-rcin


Tools/autotest/sim_vehicle.py -v ArduCopter -f + -I0 --sysid 1 --use-dir=FCU1 --add-param-file=$(pwd)/2rm.parm -A "--serial1=tcp:5761 " --add-param-file=$(pwd)/2rm.parm --console --map --no-rcin
