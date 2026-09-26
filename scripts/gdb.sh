#!/bin/bash
set -euo pipefail

ELF="build/tests/f411-hal.elf"
GDB_PORT=3333
RTT_PORT=9090

open_terminal() {
    local cmd="$1"
    if [ -n "${TERMINAL:-}" ] && command -v "$TERMINAL" >/dev/null 2>&1; then
        "$TERMINAL" -e bash -c "$cmd" 2>/dev/null && return
    fi
    for term in x-terminal-emulator gnome-terminal konsole xfce4-terminal kgx xterm alacritty; do
        if command -v "$term" >/dev/null 2>&1; then
            case "$term" in
                gnome-terminal|xfce4-terminal) "$term" -- bash -c "$cmd" ;;
                *) "$term" -e bash -c "$cmd" ;;
            esac
            return
        fi
    done
    return 1
}

cmake --build --preset tests
cmake --build --preset tests --target flash

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

(
    for i in $(seq 1 100); do
        nc -z localhost "$RTT_PORT" 2>/dev/null && break
        sleep 0.1
    done
    RTT_CMD="echo 'RTT channel 0:'; nc localhost $RTT_PORT; exec bash"
    open_terminal "$RTT_CMD" || echo "Could not auto-open RTT terminal; run: nc localhost $RTT_PORT" >&2
) &

arm-none-eabi-gdb "$ELF" -x .gdbinit
