#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Help text
show_help() {
    cat << EOF
Usage: $0 <command> [arguments]

Commands:
    flash [--fc <type>] [--port <device>] [--config <name|path>]
                               Flash firmware only (default FC: betafpv, options: betafpv, axis, lionbee, halo, chimera)
                               LionBee configs: lionbee/default (default), mightycam, or a config file path
    vtx [--port <device>] <band> <ch> [power] Flash VTX binary, configure VTX, then flash firmware + config (betafpv only)

Examples:
    $0 flash                  # Just flash firmware (betafpv)
    $0 flash --fc axis        # Flash firmware for axis
    $0 flash --fc lionbee     # Flash firmware for lionbee with default config
    $0 flash --fc lionbee --config mightycam
                               # Flash lionbee with mightycam config
    $0 flash --fc halo        # Flash firmware for HDZero Halo
    $0 flash --fc chimera     # Flash firmware for iFlight Chimera7 / Blitz F722
    $0 flash --port /dev/tty.usbmodem1234
                               # Flash using a specific serial port
    $0 vtx --port /dev/tty.usbmodem1234 5 1
                               # Full VTX setup using a specific serial port
    $0 vtx 5 1                # Full VTX setup with band R (Raceband), channel 1
    $0 vtx 5 1 3              # VTX setup with band R, channel 1, power 3 (200mW)

VTX Arguments:
    band       VTX band (1-6: 1=A, 2=B, 3=E, 4=F, 5=R, 6=L)
    channel    VTX channel (1-8)
    power      VTX power level (0-5, optional: 0=none, 1=25mW, 2=100mW, 3=200mW, 4=400mW, 5=PIT)

EOF
}

resolve_lionbee_config() {
    case "$1" in
        ""|default|lionbee|non-mighty|non_mighty)
            echo "$SCRIPT_DIR/../config/lionbee.txt"
            ;;
        mighty|mightycam|lionbee-mightycam|lionbee-mightycam.txt)
            echo "$SCRIPT_DIR/../config/lionbee-mightycam.txt"
            ;;
        /*)
            echo "$1"
            ;;
        */*)
            echo "$1"
            ;;
        *)
            echo "$SCRIPT_DIR/../config/$1"
            ;;
    esac
}

cmd_flash() {
    local fc_type="betafpv"
    local cli_port=""
    local config_name=""
    local cli_args=()

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --fc)
                if [[ -z "$2" ]]; then
                    echo "Error: --fc requires an argument"
                    exit 1
                fi
                fc_type="$2"
                shift 2
                ;;
            --port)
                if [[ -z "$2" ]]; then
                    echo "Error: --port requires an argument"
                    exit 1
                fi
                cli_port="$2"
                shift 2
                ;;
            --config)
                if [[ -z "$2" ]]; then
                    echo "Error: --config requires an argument"
                    exit 1
                fi
                config_name="$2"
                shift 2
                ;;
            *)
                echo "Error: Unknown option '$1'"
                show_help
                exit 1
                ;;
        esac
    done

    if [[ -n "$cli_port" ]]; then
        cli_args=(-p "$cli_port")
    fi
    
    echo "=========================================="
    echo "Flashing firmware for $fc_type..."
    if [[ -n "$cli_port" ]]; then
        echo "Using serial port $cli_port"
    fi
    echo "=========================================="
    
    local hex_file
    case "$fc_type" in
        betafpv)
            hex_file="$SCRIPT_DIR/../../obj/betaflight_4.5.2_STM32G47X_BETAFPVG473.hex"
            ;;
        axis)
            hex_file="$SCRIPT_DIR/../../obj/betaflight_4.5.2_STM32F7X2_AXISFLYINGF7AIO.hex"
            ;;
        lionbee)
            hex_file="$SCRIPT_DIR/../../obj/betaflight_4.5.2_LIONBEE_V2_REVB.hex"
            ;;
        halo)
            hex_file="$SCRIPT_DIR/../../obj/betaflight_4.5.2_STM32H743_HDZERO_HALO.hex"
            ;;
        chimera)
            hex_file="$SCRIPT_DIR/../../obj/betaflight_4.5.2_STM32F7X2_IFLIGHT_BLITZ_F722.hex"
            ;;
        *)
            echo "Error: Unknown FC type '$fc_type'. Must be 'betafpv', 'axis', 'lionbee', 'halo', or 'chimera'."
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
        lionbee)
            config_file="$(resolve_lionbee_config "$config_name")"
            ;;
        halo)
            config_file="$SCRIPT_DIR/../config/halo.txt"
            ;;
        chimera)
            config_file="$SCRIPT_DIR/../config/chimera.txt"
            ;;
    esac

    if [[ -n "$config_name" && "$fc_type" != "lionbee" ]]; then
        echo "Error: --config is only supported with --fc lionbee"
        exit 1
    fi

    if [[ ! -f "$config_file" ]]; then
        echo "Error: Config file not found: $config_file"
        exit 1
    fi

    if [[ ! -f "$hex_file" ]]; then
        echo "Error: Firmware hex not found: $hex_file"
        if [[ "$fc_type" == "chimera" ]]; then
            echo "Build it first with: make CONFIG=IFLIGHT_BLITZ_F722"
        fi
        exit 1
    fi

    local firmware_bin_dir="$SCRIPT_DIR/../bin"
    local firmware_bin="$firmware_bin_dir/firmware.bin"

    echo "Using config $config_file"
    mkdir -p "$firmware_bin_dir"
    
    arm-none-eabi-objcopy -I ihex -O binary "$hex_file" "$firmware_bin" || exit 1
    python3 "$SCRIPT_DIR/betaflight_cli.py" "${cli_args[@]}" -x bl || exit 1
    sleep 1
    dfu-util -a 0 -s 0x08000000:leave -D "$firmware_bin" || exit 1
    sleep 5
    python3 "$SCRIPT_DIR/betaflight_cli.py" "${cli_args[@]}" -f "$config_file" || exit 1
    
    echo "=========================================="
    echo "Done!"
    echo "=========================================="
}

cmd_vtx() {
    local cli_port=""
    local cli_args=()

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --port)
                if [[ -z "$2" ]]; then
                    echo "Error: --port requires an argument"
                    exit 1
                fi
                cli_port="$2"
                shift 2
                ;;
            *)
                break
                ;;
        esac
    done

    if [[ $# -lt 2 ]] || [[ $# -gt 3 ]]; then
        echo "Error: vtx command requires 2-3 arguments (band, channel, and optional power)"
        show_help
        exit 1
    fi

    if [[ -n "$cli_port" ]]; then
        cli_args=(-p "$cli_port")
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
    if [[ -n "$cli_port" ]]; then
        echo "Using serial port $cli_port"
    fi
    echo "=========================================="
    
    python3 "$SCRIPT_DIR/betaflight_cli.py" "${cli_args[@]}" -x bl
    sleep 1
    dfu-util -a 0 -s 0x08000000:leave -D "$SCRIPT_DIR/../bin/betafpv.bin"
    sleep 3
    if [[ -n "$VTX_POWER" ]]; then
        python3 "$SCRIPT_DIR/betaflight_cli.py" "${cli_args[@]}" -b "$VTX_BAND" -c "$VTX_CHANNEL" -w "$VTX_POWER"
    else
        python3 "$SCRIPT_DIR/betaflight_cli.py" "${cli_args[@]}" -b "$VTX_BAND" -c "$VTX_CHANNEL"
    fi
    sleep 2
    arm-none-eabi-objcopy -I ihex -O binary $SCRIPT_DIR/../../obj/betaflight_4.5.2_STM32G47X_BETAFPVG473.hex $SCRIPT_DIR/../bin/firmware.bin
    python3 "$SCRIPT_DIR/betaflight_cli.py" "${cli_args[@]}" -x bl
    sleep 1
    dfu-util -a 0 -s 0x08000000:leave -D "$SCRIPT_DIR/../bin/firmware.bin"
    sleep 3
    python3 "$SCRIPT_DIR/betaflight_cli.py" "${cli_args[@]}" -f "$SCRIPT_DIR/../config/whoop-of.txt"
    
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
