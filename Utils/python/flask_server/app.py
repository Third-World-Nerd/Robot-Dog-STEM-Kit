from flask import Flask, request, jsonify
import sys
import os
import time
import threading




# Add robot libraries to path
try:
    config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
    sys.path.append(config_dir)
    libraries_dir = os.path.abspath(os.path.join(__file__, "../libraries/python"))
    sys.path.append(libraries_dir)

    # Import your Walk class
    from config import BAUD_RATE_REC, SERIAL_PORT_REC
    from pid_controller import PIDController
    from walk import Walk
    
    from serial_comm import SerialComm
    from config import (BAUD_RATE_SEND, SERIAL_PORT_SEND, STEP_LENGTH_X, STEP_LENGTH_Y)
    
    ROBOT_AVAILABLE = True
    print("✅ Robot libraries loaded!")
except ImportError as e:
    ROBOT_AVAILABLE = False
    print(f"⚠️ Robot libraries not available: {e}")

from features import pee
app = Flask(__name__)

# Global variables
robot_walking = False
walking_thread = None
walk_instance = None

# Initialize robot
if ROBOT_AVAILABLE:
    try:
        ser = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=0.01)
        time.sleep(2)
        walk_instance = Walk(ser)
        print("✅ Robot initialized!")
    except Exception as e:
        walk_instance = None
        print(f"⚠️ Robot initialization failed: {e}")



def pid_loop(pid, leg_deltas, leg_deltas_lock):
    while True:
        loop_start = time.perf_counter()
        with leg_deltas_lock:
            new_deltas = pid.compute_leg_deltas()
            for i in range(4):
                leg_deltas[i] = new_deltas[i]
        loop_elapsed = time.perf_counter() - loop_start
        time.sleep(max(0, pid.get_dt() - loop_elapsed))





def stop_current_walk():
    """Stop any current walking"""
    global robot_walking
    robot_walking = False

def robot_forward():
    global robot_walking, walking_thread
    print("🔥 Moving robot forward")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def walk_forward():
        try:
            walk_instance.reset()
            time.sleep(1)
            walk_instance.set_step_lengthX(-120, -120)
            walk_instance.walk(duration=5)  # Walk for 5 seconds
        except Exception as e:
            print(f"Forward walk error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=walk_forward)
    walking_thread.daemon = True
    walking_thread.start()
    return "Moving forward"

def robot_backward():
    global robot_walking, walking_thread
    print("⬅️ Moving robot backward")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def walk_backward():
        try:
            walk_instance.reset()
            time.sleep(1)
            walk_instance.set_step_lengthX(120,120)
            # walk_instance.set_step_lengthY(0.0, 0.0)
            walk_instance.walk(duration=5)
        except Exception as e:
            print(f"Backward walk error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=walk_backward)
    walking_thread.daemon = True
    walking_thread.start()
    return "Moving backward"

def robot_left():
    global robot_walking, walking_thread
    print("↩️ Turning robot left")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def walk_left():
        try:
            walk_instance.reset()
            time.sleep(1)
            walk_instance.set_step_lengthX(80, -80)  # Left slower
            # walk_instance.set_step_lengthY(-80, 80)
            walk_instance.walk(duration=3)
        except Exception as e:
            print(f"Left turn error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=walk_left)
    walking_thread.daemon = True
    walking_thread.start()
    return "Turning left"

def robot_right():
    global robot_walking, walking_thread
    print("↪️ Turning robot right")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def walk_right():
        try:
            walk_instance.reset()
            time.sleep(1)
            walk_instance.set_step_lengthX(-80,80)  # Left slower
            # walk_instance.set_step_lengthY(80,-80)
            walk_instance.walk(duration=3)
            walk_instance.walk(duration=3)
        except Exception as e:
            print(f"Right turn error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=walk_right)
    walking_thread.daemon = True
    walking_thread.start()
    return "Turning right"

def robot_stop():
    global robot_walking
    print("🛑 Stopping robot")
    robot_walking=False
    if walk_instance:
        walk_instance.stop()
    return "Stopped"

# ========== CUSTOM FUNCTIONS ==========
# Add your custom functions here

def robot_pee():
    """Make the robot pee (lift leg like a dog)"""
    global robot_walking, walking_thread
    print("🐕 Robot peeing")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def pee_sequence():
        try:
            # Use your existing pee function from features.py
            # You can adjust motorPWM and duration as needed
            pee(motorPWM=100, duration=3.0, ser_write_command=ser)
        except Exception as e:
            print(f"Pee error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=pee_sequence)
    walking_thread.daemon = True
    walking_thread.start()
    return "Peeing!"

def robot_circle():
    """Make the robot walk in a circle"""
    global robot_walking, walking_thread
    print("⭕ Walking in circle")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def circle_walk():
        try:
            walk_instance.reset()
            time.sleep(1)
            # Continuous turn while walking forward
            walk_instance.set_step_lengthX(-100, -40)  # Different speeds for each leg
            walk_instance.walk(duration=8)
        except Exception as e:
            print(f"Circle walk error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=circle_walk)
    walking_thread.daemon = True
    walking_thread.start()
    return "Walking in circle"

def robot_wiggle():
    """Make the robot wiggle"""
    global robot_walking, walking_thread
    print("🎵 Robot wiggling")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def wiggle_sequence():
        try:
            walk_instance.reset()
            time.sleep(1)
            # Quick left-right wiggle motion
            for i in range(6):
                walk_instance.set_step_lengthY(-40, 40)
                walk_instance.walk(duration=0.5)
                walk_instance.set_step_lengthY(40, -40)
                walk_instance.walk(duration=0.5)
        except Exception as e:
            print(f"Wiggle error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=wiggle_sequence)
    walking_thread.daemon = True
    walking_thread.start()
    return "Wiggling!"

def robot_patrol():
    """Make the robot patrol (forward, turn around, forward, turn around)"""
    global robot_walking, walking_thread
    print("🚶 Robot patrolling")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def patrol_sequence():
        try:
            walk_instance.reset()
            time.sleep(1)
            # Patrol pattern: forward, turn 180, forward, turn 180
            for i in range(2):
                # Walk forward
                walk_instance.set_step_lengthX(-120, -120)
                walk_instance.walk(duration=3)
                time.sleep(0.5)
                # Turn around (180 degrees)
                walk_instance.set_step_lengthX(-80, 80)
                walk_instance.walk(duration=4)
                time.sleep(0.5)
        except Exception as e:
            print(f"Patrol error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=patrol_sequence)
    walking_thread.daemon = True
    walking_thread.start()
    return "Patrolling!"

# ========== WEB INTERFACE ==========

@app.route('/')
def index():
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Enhanced Robot Controller</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { 
                font-family: Arial, sans-serif; 
                text-align: center; 
                padding: 20px; 
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: white;
                min-height: 100vh;
                margin: 0;
            }
            .container { max-width: 600px; margin: 0 auto; }
            .controls { margin: 20px 0; }
            .section-title { 
                font-size: 18px; 
                margin: 20px 0 10px 0; 
                color: #fff; 
                text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
            }
            button { 
                width: 90px; height: 70px; margin: 8px; 
                font-size: 14px; border-radius: 12px;
                background: #4CAF50; color: white; border: none;
                box-shadow: 0 4px 8px rgba(0,0,0,0.2);
                cursor: pointer;
                transition: all 0.2s;
            }
            button:hover { 
                transform: translateY(-2px);
                box-shadow: 0 6px 12px rgba(0,0,0,0.3);
            }
            button:active { 
                transform: translateY(0px);
                background: #45a049; 
            }
            .basic-controls button { background: #4CAF50; }
            .custom-controls button { background: #FF6B6B; }
            .custom-controls button:hover { background: #FF5252; }
            #stop { background: #f44336 !important; }
            #stop:hover { background: #d32f2f !important; }
            .row { display: flex; justify-content: center; flex-wrap: wrap; }
            .custom-row { 
                display: grid; 
                grid-template-columns: repeat(auto-fit, minmax(100px, 1fr)); 
                gap: 10px; 
                max-width: 500px; 
                margin: 0 auto; 
            }
            #status { 
                margin-top: 20px; 
                color: #fff; 
                background: rgba(255,255,255,0.1);
                padding: 10px;
                border-radius: 8px;
                font-weight: bold;
            }
            h1 { 
                text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
                margin-bottom: 30px;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🤖 Enhanced Robot Controller</h1>
            
            <!-- Basic Controls -->
            <div class="controls basic-controls">
                <div class="section-title">Basic Movement</div>
                <div class="row">
                    <button onclick="sendCommand('forward')">↑<br>Forward</button>
                </div>
                <div class="row">
                    <button onclick="sendCommand('left')">←<br>Left</button>
                    <button id="stop" onclick="sendCommand('stop')">⏹<br>Stop</button>
                    <button onclick="sendCommand('right')">→<br>Right</button>
                </div>
                <div class="row">
                    <button onclick="sendCommand('backward')">↓<br>Backward</button>
                </div>
            </div>

            <!-- Custom Controls -->
            <div class="controls custom-controls">
                <div class="section-title">Special Moves</div>
                <div class="custom-row">
                    <button onclick="sendCommand('pee')">🐕<br>Pee</button>
                    <button onclick="sendCommand('circle')">⭕<br>Circle</button>
                    <button onclick="sendCommand('wiggle')">🎵<br>Wiggle</button>
                    <button onclick="sendCommand('patrol')">🚶<br>Patrol</button>
                </div>
            </div>

            <div id="status">Ready</div>
        </div>

        <script>
            function sendCommand(direction) {
                const statusDiv = document.getElementById('status');
                statusDiv.innerHTML = '⏳ Sending: ' + direction.charAt(0).toUpperCase() + direction.slice(1);
                
                fetch('/move', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: 'direction=' + direction
                })
                .then(response => response.text())
                .then(data => {
                    statusDiv.innerHTML = '✅ ' + data;
                })
                .catch(error => {
                    statusDiv.innerHTML = '❌ Error: ' + error;
                });
            }

            // Add keyboard support
            document.addEventListener('keydown', function(event) {
                switch(event.key) {
                    case 'ArrowUp':
                    case 'w':
                        sendCommand('forward');
                        break;
                    case 'ArrowDown':
                    case 's':
                        sendCommand('backward');
                        break;
                    case 'ArrowLeft':
                    case 'a':
                        sendCommand('left');
                        break;
                    case 'ArrowRight':
                    case 'd':
                        sendCommand('right');
                        break;
                    case ' ':
                        event.preventDefault();
                        sendCommand('stop');
                        break;
                    case '1':
                        sendCommand('pee');
                        break;
                    case '2':
                        sendCommand('circle');
                        break;
                    case '3':
                        sendCommand('wiggle');
                        break;
                    case '4':
                        sendCommand('patrol');
                        break;
                }
            });
        </script>
    </body>
    </html>
    '''

@app.route('/move', methods=['POST'])
def move():
    direction = request.form.get('direction', 'unknown')
    print(f"📡 Command: {direction.upper()}")
    
    # Execute robot actions
    if direction == 'forward':
        result = robot_forward()
    elif direction == 'backward':
        result = robot_backward()
    elif direction == 'left':
        result = robot_left()
    elif direction == 'right':
        result = robot_right()
    elif direction == 'stop':
        result = robot_stop()
    # Custom commands
    elif direction == 'pee':
        result = robot_pee()
    elif direction == 'circle':
        result = robot_circle()
    elif direction == 'wiggle':
        result = robot_wiggle()
    elif direction == 'patrol':
        result = robot_patrol()
    else:
        result = f"Unknown command: {direction}"
    
    print(f"✅ Result: {result}")
    return result

# ========== ADDITIONAL ROUTES FOR CUSTOM FUNCTIONS ==========
# You can also create dedicated routes if needed

@app.route('/custom/<action>')
def custom_action(action):
    """Handle custom actions via GET requests"""
    print(f"🎯 Custom action: {action}")
    
    if action == 'pee':
        result = robot_pee()
    elif action == 'circle':
        result = robot_circle()
    elif action == 'wiggle':
        result = robot_wiggle()
    elif action == 'patrol':
        result = robot_patrol()
    else:
        result = f"Unknown custom action: {action}"
    
    return jsonify({"status": result})

if __name__ == '__main__':
    print("🚀 Starting Enhanced Robot Controller...")
    print("🌐 Access at: http://localhost:5000")
    print("⌨️  Keyboard controls:")
    print("   Arrow keys / WASD: Basic movement")
    print("   Spacebar: Stop")
    print("   1-4: Special moves")
    
   
    ser_read_imu = SerialComm(port=SERIAL_PORT_REC, baudrate=BAUD_RATE_REC)
    pid = PIDController(ser_read_imu)
    walk_instance = Walk(ser)

    leg_deltas = [[0, 0] for _ in range(4)]
    leg_deltas_lock = threading.Lock()

    threading.Thread(target=pid_loop, args=(pid, leg_deltas, leg_deltas_lock), daemon=True).start()

    walk_instance.reset_dynamic(leg_deltas, duration=5, update_foot_position_walk=True)

    walk_instance.reset()



    
    try:
        app.run(debug=False, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        stop_current_walk()
        print("👋 Goodbye!")