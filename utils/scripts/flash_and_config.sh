#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Help text
show_help() {
    cat << EOF
Usage: $0 <command> [arguments]

Commands:
    flash [--fc <type>]          Flash firmware only (default FC: betafpv, options: betafpv, axis)
    vtx <band> <ch> [power]      Flash VTX binary, configure VTX, then flash firmware + config (betafpv only)

Examples:
    $0 flash                  # Just flash firmware (betafpv)
    $0 flash --fc axis        # Flash firmware for axis
    $0 vtx 5 1                # Full VTX setup with band R (Raceband), channel 1
    $0 vtx 5 1 3              # VTX setup with band R, channel 1, power 3 (200mW)

VTX Arguments:
    band       VTX band (1-6: 1=A, 2=B, 3=E, 4=F, 5=R, 6=L)
    channel    VTX channel (1-8)
    power      VTX power level (0-5, optional: 0=none, 1=25mW, 2=100mW, 3=200mW, 4=400mW, 5=PIT)

EOF
}

cmd_flash() {
    # Parse --fc option for flash command
    local fc_type="betafpv"
    if [[ "$1" == "--fc" ]]; then
        if [[ -z "$2" ]]; then
            echo "Error: --fc requires an argument"
            exit 1
        fi
        fc_type="$2"
        shift 2
    fi
    
    echo "=========================================="
    echo "Flashing firmware for $fc_type..."
    echo "=========================================="
    
    local hex_file
    case "$fc_type" in
        betafpv)
            hex_file="$SCRIPT_DIR/../../obj/betaflight_4.5.2_STM32G47X_BETAFPVG473.hex"
            ;;
        axis)
            hex_file="$SCRIPT_DIR/../../obj/betaflight_4.5.2_STM32F7X2_AXISFLYINGF7AIO.hex"
            ;;
        *)
            echo "Error: Unknown FC type '$fc_type'. Must be 'betafpv' or 'axis'."
            exit 1
            ;;
    esac
    
    local config_file
    case "$fc_type" in
        betafpv)
            config_file="$SCRIPT_DIR/../config/whoop-of.txt"
            ;;
        axis)
            config_file="$SCRIPT_DIR/../config/axis-of.txt"
            ;;
    esac
    
    arm-none-eabi-objcopy -I ihex -O binary "$hex_file" $SCRIPT_DIR/../bin/firmware.bin
    python3 "$SCRIPT_DIR/betaflight_cli.py" -x bl
    sleep 1
    dfu-util -a 0 -s 0x08000000:leave -D "$SCRIPT_DIR/../bin/firmware.bin"
    sleep 3
    python3 "$SCRIPT_DIR/betaflight_cli.py" -f "$config_file"
    
    echo "=========================================="
    echo "Done!"
    echo "=========================================="
}

cmd_vtx() {
    if [[ $# -lt 2 ]] || [[ $# -gt 3 ]]; then
        echo "Error: vtx command requires 2-3 arguments (band, channel, and optional power)"
        show_help
        exit 1
    fi
    
    VTX_BAND="$1"
    VTX_CHANNEL="$2"
    VTX_POWER="$3"
    
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
    
    # Validate power (0-5) if provided
    if [[ -n "$VTX_POWER" ]] && ! [[ "$VTX_POWER" =~ ^[0-5]$ ]]; then
        echo "Error: Power must be 0-5 (0=none, 1=25mW, 2=100mW, 3=200mW, 4=400mW, 5=PIT)"
        exit 1
    fi
    
    echo "=========================================="
    if [[ -n "$VTX_POWER" ]]; then
        echo "Configuring VTX: Band $VTX_BAND, Channel $VTX_CHANNEL, Power $VTX_POWER"
    else
        echo "Configuring VTX: Band $VTX_BAND, Channel $VTX_CHANNEL"
    fi
    echo "=========================================="
    
    python3 "$SCRIPT_DIR/betaflight_cli.py" -x bl
    sleep 1
    dfu-util -a 0 -s 0x08000000:leave -D "$SCRIPT_DIR/../bin/betafpv.bin"
    sleep 3
    if [[ -n "$VTX_POWER" ]]; then
        python3 "$SCRIPT_DIR/betaflight_cli.py" -b "$VTX_BAND" -c "$VTX_CHANNEL" -w "$VTX_POWER"
    else
        python3 "$SCRIPT_DIR/betaflight_cli.py" -b "$VTX_BAND" -c "$VTX_CHANNEL"
    fi
    sleep 2
    arm-none-eabi-objcopy -I ihex -O binary $SCRIPT_DIR/../../obj/betaflight_4.5.2_STM32G47X_BETAFPVG473.hex $SCRIPT_DIR/../bin/firmware.bin
    python3 "$SCRIPT_DIR/betaflight_cli.py" -x bl
    sleep 1
    dfu-util -a 0 -s 0x08000000:leave -D "$SCRIPT_DIR/../bin/firmware.bin"
    sleep 3
    python3 "$SCRIPT_DIR/betaflight_cli.py" -f "$SCRIPT_DIR/../config/whoop-of.txt"
    
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

