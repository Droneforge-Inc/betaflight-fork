#!/usr/bin/env python3
"""
Betaflight CLI Communication Script
Communicate with flight controller via serial CLI interface

Usage:
    # Auto-detect port and set VTX (band, channel, power)
    ./betaflight_cli.py -b 5 -c 1 -w 3
    
    # Run commands
    ./betaflight_cli.py -x "version" -x "get name"
    
    # Interactive mode
    ./betaflight_cli.py -i
    
    # Specify port manually
    ./betaflight_cli.py -p /dev/ttyUSB0 -x "status"
"""

import serial
import serial.tools.list_ports
import argparse
import sys
import time
import threading


class BetaflightCLI:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.in_cli = False
        self.read_thread = None
        self.running = False
        self.fc_rebooting = False  # Track if FC is rebooting
        
    def connect(self):
        """Open serial connection to FC"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            time.sleep(0.1)  # Let connection stabilize
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        try:
            if self.ser:
                if self.ser.is_open and self.in_cli and not self.fc_rebooting:
                    self.exit_cli()
                if self.ser.is_open:
                    self.ser.close()
        except Exception as e:
            # Log but don't crash on disconnect errors
            if not self.fc_rebooting:
                print(f"Warning during disconnect: {e}")
        finally:
            # Always print status
            if self.fc_rebooting:
                print("Done (FC rebooted)")
            else:
                print("Disconnected")
    
    def enter_cli(self):
        """Enter CLI mode by sending '#' character"""
        if not self.ser or not self.ser.is_open:
            return False
        
        # Clear any existing data
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
        # Send '#' to enter CLI mode
        self.ser.write(b'#')
        time.sleep(0.3)
        
        # Read response
        response = self._read_until_prompt(timeout=2)
        
        if response:
            self.in_cli = True
            print("Entered CLI mode")
            return True
        else:
            print("Failed to enter CLI mode")
            return False
    
    def exit_cli(self):
        """Exit CLI mode"""
        if self.in_cli:
            try:
                self.send_command("exit")
                time.sleep(0.5)
            except Exception as e:
                # Log but don't crash
                print(f"Warning during exit: {e}")
            finally:
                self.in_cli = False
    
    def send_command(self, command):
        """Send a command to the CLI"""
        if not self.ser or not self.ser.is_open:
            print("Not connected")
            return None
        
        # Detect commands that cause FC to reboot
        cmd_lower = command.strip().lower()
        if cmd_lower in ['save', 'bl', 'exit']:
            self.fc_rebooting = True
        
        try:
            # Add newline if not present
            if not command.endswith('\n'):
                command += '\r\n'
            
            self.ser.write(command.encode('utf-8'))
            
            # Read response
            timeout = 5 if self.fc_rebooting else 3
            response = self._read_until_prompt(timeout=timeout)
            return response
        except Exception as e:
            # Log but don't crash on write errors (FC might have rebooted)
            if not self.fc_rebooting:
                print(f"Error sending command: {e}")
            return None
    
    def _read_until_prompt(self, timeout=2):
        """Read until CLI prompt '# ' is received"""
        response = ""
        start_time = time.time()
        
        try:
            while time.time() - start_time < timeout:
                try:
                    if self.ser.in_waiting > 0:
                        chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                        response += chunk
                        
                        # Check for CLI prompt
                        if '# ' in response or response.endswith('#'):
                            break
                except Exception:
                    # Connection might be lost (FC rebooted), exit quietly
                    break
                time.sleep(0.01)
        except Exception:
            # Catch any other errors (e.g., serial port access)
            pass
        
        return response
    
    def interactive_mode(self):
        """Interactive CLI session"""
        if not self.in_cli:
            if not self.enter_cli():
                return
        
        print("\nInteractive CLI mode - type 'exit' to quit")
        print("=" * 50)
        
        try:
            while True:
                try:
                    cmd = input()
                    if not cmd:
                        continue
                    
                    if cmd.lower() in ['exit', 'quit']:
                        break
                    
                    response = self.send_command(cmd)
                    if response:
                        # Print without the command echo and prompt
                        lines = response.split('\n')
                        for line in lines:
                            line = line.strip()
                            if line and line != '#' and not line.startswith('#'):
                                print(line)
                
                except KeyboardInterrupt:
                    print("\nInterrupted")
                    break
                except EOFError:
                    break
                except Exception as e:
                    print(f"Error: {e}")
        
        finally:
            if not self.fc_rebooting:
                self.exit_cli()
    
    def execute_commands(self, commands):
        """Execute a list of commands"""
        if not self.in_cli:
            if not self.enter_cli():
                return
        
        try:
            for cmd in commands:
                print(f"\n> {cmd}")
                response = self.send_command(cmd)
                if response:
                    # Clean up response
                    lines = response.split('\n')
                    for line in lines:
                        line = line.strip()
                        if line and line != '#' and not line.startswith(cmd):
                            print(line)
        except Exception as e:
            # Log but don't crash
            print(f"Error executing commands: {e}")
        finally:
            # Always try to exit
            if not self.fc_rebooting:
                self.exit_cli()


def list_ports():
    """List available serial ports"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found")
        return []
    
    print("\nAvailable serial ports:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")
    return ports


def auto_detect_port():
    """Auto-detect Betaflight FC port"""
    ports = serial.tools.list_ports.comports()
    
    # Look for common FC identifiers
    for port in ports:
        desc_lower = port.description.lower()
        # Check for common STM32/Betaflight indicators
        if any(x in desc_lower for x in ['stm32', 'betaflight', 'flight controller', 'serial', 'usb']):
            print(f"Auto-detected FC on: {port.device} - {port.description}")
            return port.device
    
    # If no specific match, try first available port
    if ports:
        print(f"Using first available port: {ports[0].device} - {ports[0].description}")
        return ports[0].device
    
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Betaflight CLI Communication Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-detect and run commands
  %(prog)s -x "version" -x "get name"
  
  # Set VTX band (R) and channel 1, auto-saves
  %(prog)s -b 5 -c 1
  
  # Set VTX band, channel, and power (200mW), auto-saves
  %(prog)s -b 5 -c 1 -w 3
  
  # Set VTX and run additional commands
  %(prog)s -b 3 -c 4 -x "get vtx"
  
  # Interactive mode (auto-detects port)
  %(prog)s -i
  
  # Specify port manually
  %(prog)s -p /dev/ttyUSB0 -x "status"
  
  # Run commands from file
  %(prog)s -f setup_commands.txt
  
  # List available ports
  %(prog)s --list

VTX Bands: 1=A, 2=B, 3=E, 4=F, 5=R (Raceband) 6=L (Lowband)
        """
    )
    
    parser.add_argument('-p', '--port', help='Serial port (e.g., /dev/ttyUSB0, COM3). Auto-detects if not specified')
    parser.add_argument('-x', '--command', action='append', help='Command to execute (can be used multiple times)')
    parser.add_argument('-f', '--file', help='File containing commands (one per line)')
    parser.add_argument('-i', '--interactive', action='store_true', help='Interactive CLI mode')
    parser.add_argument('-l', '--list', action='store_true', help='List available serial ports')
    parser.add_argument('-b', '--vtx-band', type=int, choices=range(1, 7), metavar='1-6', help='Set VTX band (1=A, 2=B, 3=E, 4=F, 5=R, 6=L)')
    parser.add_argument('-c', '--vtx-channel', type=int, choices=range(1, 9), metavar='1-8', help='Set VTX channel (1-8)')
    parser.add_argument('-w', '--vtx-power', type=int, choices=range(0, 6), metavar='0-5', help='Set VTX power level (0=none, 1=25mW, 2=100mW, 3=200mW, 4=400mW, 5=PIT)')
    
    args = parser.parse_args()
    
    # List ports if requested
    if args.list:
        list_ports()
        return 0
    
    # Auto-detect port if not specified
    if not args.port:
        args.port = auto_detect_port()
        if not args.port:
            print("Error: No serial port detected. Use -p to specify manually or --list to see available ports")
            return 1
    
    # Create CLI interface (always 115200 baud)
    cli = BetaflightCLI(args.port, baudrate=115200)
    
    try:
        # Connect
        if not cli.connect():
            return 1
        
        # Interactive mode if requested
        if args.interactive:
            cli.interactive_mode()
        
        # Execute commands from file if provided
        elif args.file:
            try:
                with open(args.file, 'r') as f:
                    file_commands = [line.strip() for line in f if line.strip() and not line.startswith('#')]
                # Always save after all commands
                file_commands.append("save")
                cli.execute_commands(file_commands)
            except FileNotFoundError:
                print(f"Error: File '{args.file}' not found")
                return 1
        
        # Execute commands from arguments
        elif args.command:
            commands = list(args.command)
            # Always save after all commands
            commands.append("save")
            cli.execute_commands(commands)
        
        # Default: VTX commands only if nothing else specified
        elif args.vtx_band is not None or args.vtx_channel is not None or args.vtx_power is not None:
            commands = []
            if args.vtx_band is not None:
                commands.append(f"set vtx_band = {args.vtx_band}")
            if args.vtx_channel is not None:
                commands.append(f"set vtx_channel = {args.vtx_channel}")
            if args.vtx_power is not None:
                commands.append(f"set vtx_power = {args.vtx_power}")
            # Always save after all commands
            commands.append("save")
            cli.execute_commands(commands)
        
        # No commands specified at all
        else:
            print("Error: No commands specified. Use -x, -f, -b/-c/-w (VTX), or -i for interactive mode")
            return 1
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 1
    except Exception as e:
        # Log error but don't show stack trace
        print(f"Error: {e}")
        return 1
    finally:
        # Always try to disconnect cleanly
        try:
            cli.disconnect()
        except Exception as e:
            # Don't crash on disconnect errors
            print(f"Warning: {e}")
    
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as e:
        # Catch any unhandled exceptions at top level
        print(f"Fatal error: {e}")
        sys.exit(1)

