import threading
import time
import websocket
import json
import curses

# Initialize variables
x, y = 0, 0
lock = threading.Lock()
is_running = True

def display_values(stdscr):
    stdscr.clear()
    stdscr.addstr("Control the values using keyboard keys. Press 'ESC' to exit.\n")
    stdscr.addstr(f'X: {x:.2f} Y: {y:.2f}\n')
    stdscr.refresh()

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def update_values(stdscr):
    global x, y, is_running
    stdscr.nodelay(True)

    key_actions = {
        'w': lambda: assign('y', y + 0.01),
        's': lambda: assign('y', y - 0.01),
        'a': lambda: assign('x', x - 0.01),
        'd': lambda: assign('x', x + 0.01),
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
    global x, y
    if var_name == 'x':
        x = clamp(value, -1, 1)
    elif var_name == 'y':
        y = clamp(value, -1, 1)
    elif var_name == 'z':
        z = clamp(value, -1, 1)

# Function to handle data sending
def send_data(ws):
    global x, y, z, pitch, yaw, roll
    while is_running:
        try:
            with lock:
                data = {
                    "data": [x, y]
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

# WebSocket connection
def start_websocket():
    uri = "ws://localhost:6789"
    try:
        ws = websocket.WebSocketApp(uri, on_open=lambda ws: threading.Thread(target=send_data, args=(ws,)).start())
        ws.run_forever()
    except Exception as e:
        print(f"Error connecting to WebSocket: {e}")

# Start WebSocket thread
threading.Thread(target=start_websocket, daemon=True).start()

# Start curses application
curses.wrapper(update_values)

# Clean up after exiting curses
is_running = False
print("Exiting application...")
