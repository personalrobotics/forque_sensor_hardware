import telnetlib
import sys

def connect_to_sensor(host, port):
    try:
        tn = telnetlib.Telnet(host, port)
        return tn
    except Exception as e:
        print(f"Error connecting to sensor: {e}")
        return None

def send_command(tn, command):
    try:
        tn.write(command.encode('ascii') + b'\r\n')
    except Exception as e:
        print(f"Error sending command: {e}")

def read_output(tn):
    try:
        output = tn.read_very_eager().decode('ascii').strip()
        if output:
            return output
    except Exception as e:
        print(f"Error reading output: {e}")
    return None

def parse_data(data_line):
    data_list = data_line.split()

     # Extract the relevant data (forces and torques) and convert to signed integers using two's complement
    force_x = int(data_list[6], 16)
    if force_x > 0x7FFFFFFF:
        force_x -= 0x100000000

    force_y = int(data_list[7], 16)
    if force_y > 0x7FFFFFFF:
        force_y -= 0x100000000

    force_z = int(data_list[8], 16)
    if force_z > 0x7FFFFFFF:
        force_z -= 0x100000000

    torque_x = int(data_list[9], 16)
    if torque_x > 0x7FFFFFFF:
        torque_x -= 0x100000000

    torque_y = int(data_list[10], 16)
    if torque_y > 0x7FFFFFFF:
        torque_y -= 0x100000000

    torque_z = int(data_list[11], 16)
    if torque_z > 0x7FFFFFFF:
        torque_z -= 0x100000000

    # Convert counts to physical units (Newtons and Newton-meters)
    force_x /= 1000000
    force_y /= 1000000
    force_z /= 1000000
    torque_x /= 1000000
    torque_y /= 1000000
    torque_z /= 1000000

    return force_x, force_y, force_z, torque_x, torque_y, torque_z

def is_hex_string(s):
    try:
        int(s, 16)
        return True
    except ValueError:
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py <sensor_ip>")
        return

    host = sys.argv[1]
    port = 23

    tn = connect_to_sensor(host, port)
    if not tn:
        return

    # Send a command to reset bias
    send_command(tn, "bias 3 on")

    # Send a command to start data streaming
    send_command(tn, 'd on')

    while True:
        output = read_output(tn)
        if output:
            if is_hex_string(output.split()[0]) and len(output.split()) == 12:
                force_x, force_y, force_z, torque_x, torque_y, torque_z = parse_data(output)

                # Print the extracted data
                print("Force X:", force_x)
                print("Force Y:", force_y)
                print("Force Z:", force_z)
                print("Torque X:", torque_x)
                print("Torque Y:", torque_y)
                print("Torque Z:", torque_z)
                print()

    # Send a command to stop data streaming
    send_command(tn, 'd off')

if __name__ == '__main__':
    main()
