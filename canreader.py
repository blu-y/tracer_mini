import can
import datetime

system_status = {"Status": "-", "Mode": "-", 
                 "Battery": "-", "Fault": "-", 
                 "Fault_info": ""}
light_stat = {"Flag": "-", "Mode": "-", "Brightness": "-"}
movement_status = {"Ctrl_Lin": "-", "Ctrl_Rot": "-", 
                   "Feedb_Lin": "-", "Feedb_Rot": "-"}
motor_status = {"Odom_L": "-", "Odom_R": "-", 
                "RPM_L": "-", "RPM_R": "-"}
last_ts = 0

def decode_message(msg):
    global system_status, light_stat, movement_status, motor_status

    if msg.arbitration_id == 0x211:
        status_map = {0x00: "Normal", 0x01: "E-Stop", 0x02: "Error"}
        mode_map = {0x00: "Standby", 0x01: "CAN", 0x03: "RC"}
        status = status_map.get(msg.data[0], "Unknown")
        mode = mode_map.get(msg.data[1], "Unknown")
        battery_voltage = (msg.data[2] << 8 | msg.data[3]) / 10.0

        fault_bits = (msg.data[4] << 8) | msg.data[5]
        fault_info = []
        if fault_bits & (1 << 0):
            fault_info.append("BATFAIL")
        if fault_bits & (1 << 1):
            fault_info.append("BATLOW")
        if fault_bits & (1 << 2):
            fault_info.append("NO RC")
        if fault_bits & (1 << 3):
            fault_info.append("MOTOR1")
        if fault_bits & (1 << 6):
            fault_info.append("MOTOR2")
        if fault_bits & (1 << 8):
            fault_info.append("MTRDRVR")

        fault = ", ".join(fault_info) if fault_info else "-"

        system_status["Status"] = status
        system_status["Mode"] = mode
        system_status["Battery"] = f"{battery_voltage:.1f}V"
        if len(fault_info) < 2:
            system_status["Fault"] = fault
            system_status["Fault_info"] = ""
        else: 
            system_status["Fault"] = "Various"
            system_status["Fault_info"] = fault

    elif msg.arbitration_id == 0x221:
        lin_feedb = int.from_bytes(msg.data[0:2], byteorder='big', signed=True)
        rot_feedb = int.from_bytes(msg.data[2:4], byteorder='big', signed=True) / 1000
        movement_status["Feedb_Lin"] = f"{lin_feedb}"
        movement_status["Feedb_Rot"] = f"{rot_feedb}"
    elif msg.arbitration_id == 0x111:
        lin_ctrl = int.from_bytes(msg.data[0:2], byteorder='big', signed=True)
        rot_ctrl = int.from_bytes(msg.data[2:4], byteorder='big', signed=True) / 1000
        movement_status["Ctrl_Lin"] = f"{lin_ctrl}"
        movement_status["Ctrl_Rot"] = f"{rot_ctrl}"
        pass
    elif msg.arbitration_id == 0x121:
        lflag_map = {0x00: "Disable", 0x01: "Enable"}
        lmode_map = {0x00: "NC", 0x01: "NO", 0x02: "BL", 0x03: "USER"}
        lflag = lflag_map.get(msg.data[0], "Unknown")
        lmode = lmode_map.get(msg.data[1], "Unknown")
        brightness = msg.data[2]
        light_stat["Flag"] = lflag
        light_stat["Mode"] = lmode
        light_stat["Brightness"] = brightness
    elif msg.arbitration_id == 0x231:
        # Update the lighting control information
        pass
    elif msg.arbitration_id == 0x421:
        # Update the control mode setting information
        pass
    elif msg.arbitration_id == 0x441:
        # Update the status position information
        pass
    elif msg.arbitration_id == 0x311:
        left_odom = int.from_bytes(msg.data[0:4], byteorder='big', signed=True)
        right_odom = int.from_bytes(msg.data[5:8], byteorder='big', signed=True)
        motor_status["Odom_L"] = f"{left_odom}"
        motor_status["Odom_R"] = f"{right_odom}"
    elif msg.arbitration_id == 0x251:
        left_rpm = int.from_bytes(msg.data[0:2], byteorder='big', signed=True)
        motor_status["RPM_L"] = f"{left_rpm}"
    elif msg.arbitration_id == 0x252:
        right_rpm = int.from_bytes(msg.data[0:2], byteorder='big', signed=True)
        motor_status["RPM_R"] = f"{right_rpm}"
    elif msg.arbitration_id == 0x261:
        # Update the left failure information
        pass
    elif msg.arbitration_id == 0x262:
        # Update the righ failure information
        pass
    else:
        return None
line = ""
def receive_can(channel, bustype):
    bus = can.interface.Bus(channel=channel, bustype=bustype)
    print("Listening for CAN messages on {}...".format(channel))
    global last_ts, line
    try:
        while True:
            msg = bus.recv()
            if msg:
                decode_message(msg)
                if msg.arbitration_id == 0x111:
                    line = f"{hex(msg.arbitration_id)}, {msg.dlc}, {msg.data.hex()}, {msg.channel}"
                if msg.timestamp - last_ts > 0.05:
                    print_cli(msg, line=line)
                    last_ts = msg.timestamp
    except KeyboardInterrupt:
        print("Stopped listening for CAN messages")

def print_cli(msg, line=""):
    time_str = datetime.datetime.fromtimestamp(msg.timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-4]
    header = f"Tracer Mini                                      {time_str}"
    sections = "─" * 75
    print(f"\033[H\033[J", end="")  # Clear the screen
    print(f"┌{sections}┐")
    print(f"│  {header}  │")
    print(f"│ ┌──[System Stat]──┬─[Movement Status]─┬─[Motor Status]─┬──[Light Stat]──┐ │")
    print(f"│ │  Status {system_status['Status']:>7} │        Lin    Rot │     Odom   RPM │   Flag {light_stat['Flag']:>7} │ │")
    print(f"│ │    Mode {system_status['Mode']:>7} │  Ctrl {movement_status['Ctrl_Lin']:>4} {movement_status['Ctrl_Rot']:>6} │  L {motor_status['Odom_L']:>5} {motor_status['RPM_L']:>5} │   Mode {light_stat['Mode']:>7} │ │")
    print(f"│ │ Battery {system_status['Battery']:>7} │ Feedb {movement_status['Feedb_Lin']:>4} {movement_status['Feedb_Rot']:>6} │  R {motor_status['Odom_R']:>5} {motor_status['RPM_R']:>5} │ Bright {light_stat['Brightness']:>7} │ │")
    print(f"│ │   Fault {system_status['Fault']:>7} │       mm/s  rad/s │       mm       │                │ │")
    print(f"│ └─────────────────┴───────────────────┴────────────────┴────────────────┘ │")
    if len(system_status['Fault_info']) > 1:
        print(f"│ {system_status['Fault_info']:<73} │")
    if len(line) > 1: print(f"│ {line:<73} │")
    print(f"└{sections}┘")

if __name__ == "__main__":
    receive_can(channel='can0', bustype='socketcan')
