#!/bin/bash
set -euo pipefail

ELF="build/debug-tests/f411-hal.elf"
GDB_PORT=3333

openocd -f interface/stlink.cfg -f target/stm32f4x.cfg > /tmp/openocd.log 2>&1 &
OPENOCD_PID=$!

cleanup() {
    kill "$OPENOCD_PID" 2>/dev/null || true
    wait "$OPENOCD_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for i in $(seq 1 50); do
    if nc -z localhost "$GDB_PORT" 2>/dev/null; then
        break
    fi
    sleep 0.1
done

if ! nc -z localhost "$GDB_PORT" 2>/dev/null; then
    echo "openocd failed to start, see /tmp/openocd.log" >&2
    cat /tmp/openocd.log >&2
    exit 1
fi

arm-none-eabi-gdb "$ELF" \
    --tui \
    -ex "target extended-remote localhost:$GDB_PORT" \
    -ex "monitor reset halt" \
    -ex "load" \
    -ex "break main" \
    -ex "monitor reset halt"
