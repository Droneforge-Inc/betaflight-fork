#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Help text
show_help() {
    cat << EOF
Usage: $0 <command> [arguments]

Commands:
    flash              Flash firmware only
    vtx <band> <ch>    Flash VTX binary, configure VTX, then flash firmware + config

Examples:
    $0 flash           # Just flash firmware
    $0 vtx 5 1         # Full VTX setup with band R (Raceband), channel 1

VTX Arguments:
    band       VTX band (1-6: 1=A, 2=B, 3=E, 4=F, 5=R, 6=L)
    channel    VTX channel (1-8)

EOF
}

cmd_flash() {
    echo "=========================================="
    echo "Flashing firmware..."
    echo "=========================================="
    
    arm-none-eabi-objcopy -I ihex -O binary $SCRIPT_DIR/../../obj/betaflight_4.5.2_STM32G47X_BETAFPVG473.hex $SCRIPT_DIR/../bin/firmware.bin
    python3 "$SCRIPT_DIR/betaflight_cli.py" -x bl
    sleep 1
    dfu-util -a 0 -s 0x08000000:leave -D "$SCRIPT_DIR/../bin/firmware.bin"
    sleep 3
    python3 "$SCRIPT_DIR/betaflight_cli.py" -f "$SCRIPT_DIR/../config/whoop.txt"
    
    echo "=========================================="
    echo "Done!"
    echo "=========================================="
}

cmd_vtx() {
    if [[ $# -ne 2 ]]; then
        echo "Error: vtx command requires 2 arguments (band and channel)"
        show_help
        exit 1
    fi
    
    VTX_BAND="$1"
    VTX_CHANNEL="$2"
    
    # Validate band (1-6)
    if ! [[ "$VTX_BAND" =~ ^[1-6]$ ]]; then
        echo "Error: Band must be 1-6"
        exit 1
    fi
    
    # Validate channel (1-8)
    if ! [[ "$VTX_CHANNEL" =~ ^[1-8]$ ]]; then
        echo "Error: Channel must be 1-8"
        exit 1
    fi
    
    echo "=========================================="
    echo "Configuring VTX: Band $VTX_BAND, Channel $VTX_CHANNEL"
    echo "=========================================="
    
    python3 "$SCRIPT_DIR/betaflight_cli.py" -x bl
    sleep 1
    dfu-util -a 0 -s 0x08000000:leave -D "$SCRIPT_DIR/../bin/betafpv.bin"
    sleep 3
    python3 "$SCRIPT_DIR/betaflight_cli.py" -b "$VTX_BAND" -c "$VTX_CHANNEL"
    sleep 2
    arm-none-eabi-objcopy -I ihex -O binary $SCRIPT_DIR/../../obj/betaflight_4.5.2_STM32G47X_BETAFPVG473.hex $SCRIPT_DIR/../bin/firmware.bin
    python3 "$SCRIPT_DIR/betaflight_cli.py" -x bl
    sleep 1
    dfu-util -a 0 -s 0x08000000:leave -D "$SCRIPT_DIR/../bin/firmware.bin"
    sleep 3
    python3 "$SCRIPT_DIR/betaflight_cli.py" -f "$SCRIPT_DIR/../config/whoop.txt"
    
    echo "=========================================="
    echo "Done!"
    echo "=========================================="
}

# Check for help flag
if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]] || [[ $# -eq 0 ]]; then
    show_help
    exit 0
fi

# Parse command
COMMAND="$1"
shift

case "$COMMAND" in
    flash)
        cmd_flash "$@"
        ;;
    vtx)
        cmd_vtx "$@"
        ;;
    *)
        echo "Error: Unknown command '$COMMAND'"
        echo ""
        show_help
        exit 1
        ;;
esac

