import socket
import argparse

def main():
    parser = argparse.ArgumentParser(description="Send command to ESP32 and optionally receive data.")
    parser.add_argument("--ip", required=True, help="IP address of ESP32")
    parser.add_argument("--port", type=int, default=1234, help="Port number (default: 1234)")
    parser.add_argument("--cmd", type=int, default=9, help="Command byte to send (default: 9)")
    args = parser.parse_args()

    try:
        with socket.create_connection((args.ip, args.port), timeout=5) as sock:
            # Send command byte
            sock.sendall(bytes([args.cmd]))
            print(f"Sent command {args.cmd} to {args.ip}:{args.port}")

            # If command is 9, receive 256 bytes
            if args.cmd == 9:
                data = sock.recv(256)
                print(f"Received {len(data)} bytes:")
                print(" ".join(f"0x{b:02X}" for b in data))
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
