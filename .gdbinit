python
import sys, os
sys.path.insert(0, os.path.expanduser("~/.gdb-scripts/PyCortexMDebug"))
import warnings
warnings.filterwarnings("ignore")
from cmdebug.svd_gdb import LoadSVD
LoadSVD()
end

target extended-remote localhost:3333
monitor reset halt
load
svd_load .vscode/STM32F411.svd
break main
continue
monitor rtt setup 0x20000000 0x20000 "SEGGER RTT"
monitor rtt start
monitor rtt server start 9090 0 # An RTT port is started on port 9090. Run `nc localhost 9090` to view the logs

define reflash
    delete
    file build/debug-tests/f411-hal.elf
    monitor reset halt
    load
    break main
    monitor reset halt
    continue
end
