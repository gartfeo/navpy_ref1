import serial
import time

# === CONFIGURATION ===
SERIAL_PORT = "COM11"  # Change to your actual serial port (e.g., /dev/ttyUSB0 for Linux)
BAUD_RATE = 115200  # Default UART baud rate

# === LIST OF QUERY COMMANDS ===
query_commands = [
    "AT",  # Basic test command
    "AT+INFO=?",  # Get module information
    "AT+DEVTYPE=?",  # Get device type
    "AT+FWCODE=?",  # Get firmware version
    "AT+POWER=?",  # Get transmit power
    "AT+CHANNEL=?",  # Get current working channel
    "AT+UART=?",  # Get UART settings
    "AT+RATE=?",  # Get air rate
    "AT+OPTION=?",  # Get communication mode
    "AT+PANID=?",  # Get network ID
    "AT+TYPE=?",  # Get node type (Router/Terminal)
    "AT+SRC_ADDR=?",  # Get source address
    "AT+DST_ADDR=?",  # Get destination address
    "AT+SRC_PORT=?",  # Get source port
    "AT+DST_PORT=?",  # Get destination port
    "AT+HEAD=?",  # Check if frame header is enabled
    "AT+SECURITY=?",  # Check if encryption is enabled
    "AT+MAC=?",  # Get MAC address
    "AT+CSMA_RNG=?",  # Get CSMA collision avoidance time
    "AT+ROUTER_SCORE=?",  # Get routing failure threshold
    "AT+ROUTER_TIME=?",  # Get routing timeout
    "AT+FILTER_TIME=?",  # Get broadcast filtering timeout
    "AT+ACK_TIME=?",  # Get response timeout
    "AT+RESET_TIME=?",  # Get auto-reset time
    "AT+MEMBER_RAD=?",  # Get multicast radius
    "AT+NONMEMBER_RAD=?",  # Get non-member multicast radius
    "AT+GROUP_ADD=?",  # Get multicast group information
    "AT+GROUP_CLR=?",  # Get multicast group settings
    "AT+ROUTER_CLR=?",  # Get routing table information
    "AT+ROUTER_SAVE=?",  # Check if routing is saved in Flash
    "AT+ROUTER_READ=?",  # Read routing table from Flash
]

config_commands = [
    "AT+RATE=2,1",          # 7K air rate
    "AT+POWER=22,1",        # +22 dBm
    "AT+TYPE=0,1",          # Routing node
    "AT+ROUTER_TIME=10000",
    "AT+ROUTER_SCORE=10",
    "AT+ROUTER_SAVE=1",
    "AT+OPTION=3,1",        # Broadcast
    "AT+FILTER_TIME=10000",
    "AT+CSMA_RNG=200",
    "AT+SRC_ADDR=07698,1",
    "AT+DST_ADDR=FFFF,1",   # 0xFFFF broadcast
    "AT+SECURITY=1",        # Optional encryption
    "AT+ACK_TIME=5000",     # Higher unicast ack, if needed
    "AT+HEAD=1,1",          # Keep frame header
]

# === OPEN SERIAL CONNECTION ===
ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)


def send_at_command(command):
    """Send AT command and return response"""
    ser.write(command.encode())  # Send command
    time.sleep(0.3)  # Wait for response
    r = ser.read_all().decode().strip()  # Read response
    return r


try:
    # === APPLY OPTIMIZED CONFIGURATION ===
    print("\n🔹 Configuring LoRa Module for Maximum Stability...\n")

    # === VERIFY SETTINGS ===
    print("\n🔹 Verifying Configurations...")
    for cmd in query_commands:
        response = send_at_command(cmd)
        print(f"Check {cmd} → {response}")

    # for cmd in config_commands:
    #     response = send_at_command(cmd)
    #     print(f"Command: {cmd} → Response: {response}")

    # === VERIFY SETTINGS ===
    # print("\n🔹 Verifying Configurations...")
    # for cmd in query_commands:
    #     response = send_at_command(cmd)
    #     print(f"Check {cmd} → {response}")

    # === CLOSE SERIAL CONNECTION ===
    ser.close()
    print("\n✅ Configuration Complete! Your LoRa module is optimized.")

except Exception as e:
    print(f"❌ Error: {e}")
