import threading
import time
import websocket
import json
import curses

# Initialize variables
x, y, z = 0, 0, 0
pitch, yaw, roll = 0, 0, 0
lock = threading.Lock()
is_running = True


def display_values(stdscr):
    stdscr.clear()
    stdscr.addstr("Control the values using keyboard keys. Press 'ESC' to exit.\n")
    stdscr.addstr(f'X: {x:.2f} Y: {y:.2f} Z: {z:.2f} Pitch: {pitch:.2f} Yaw: {yaw:.2f} Roll: {roll:.2f}\n')
    stdscr.refresh()

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)
def update_values(stdscr):
    global x, y, z, pitch, yaw, roll, is_running
    stdscr.nodelay(True)

    key_actions = {
        'w': lambda: assign('y', y + 0.01),
        's': lambda: assign('y', y - 0.01),
        'a': lambda: assign('x', x - 0.01),
        'd': lambda: assign('x', x + 0.01),
        'o': lambda: assign('roll', roll + 0.01),
        'u': lambda: assign('roll', roll - 0.01),
        'q': lambda: assign('z', z + 0.01),
        'e': lambda: assign('z', z - 0.01),
        'i': lambda: assign('pitch', pitch + 0.01),
        'k': lambda: assign('pitch', pitch - 0.01),
        'j': lambda: assign('yaw', yaw - 0.01),
        'l': lambda: assign('yaw', yaw + 0.01),
    }

    while is_running:
        try:
            key = stdscr.getkey()
            if key == '\x1b':  # ESC key for exit
                is_running = False
            elif key in key_actions:
                key_actions[key]()
                display_values(stdscr)
        except Exception as e:
            pass
def assign(var_name, value):
    global x, y, z, pitch, yaw, roll
    if var_name == 'x':
        x = clamp(value, -1, 1)
    elif var_name == 'y':
        y = clamp(value, -1, 1)
    elif var_name == 'z':
        z = clamp(value, -1, 1)
    elif var_name == 'pitch':
        pitch = clamp(value, -1, 1)
    elif var_name == 'yaw':
        yaw = clamp(value, -1, 1)
    elif var_name == 'roll':
        roll = clamp(value, -1, 1)
def send_data(ws):
    global x, y, z, pitch, yaw, roll
    while is_running:
        try:
            with lock:
                data = {
                    "data": [x, y, z, pitch, yaw, roll]
                }
            ws.send(json.dumps(data))
        except websocket.WebSocketConnectionClosedException:
            print("WebSocket connection closed. Attempting to reconnect...")
            time.sleep(5)
            start_websocket()  # Attempt to reconnect
            break
        except Exception as e:
            print(f"Error in sending data: {e}")
            break
        time.sleep(0.1)  # Sending data at 10 Hz

def start_websocket():
    uri = "ws://localhost:6789"
    ws = websocket.WebSocketApp(uri, on_open=lambda ws: threading.Thread(target=send_data, args=(ws,)).start())
    ws.run_forever()

threading.Thread(target=start_websocket, daemon=True).start()

curses.wrapper(update_values)
is_running = False
print("Exiting application...")
