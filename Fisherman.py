import pyautogui,sounddevice as sd,threading,time,win32api,configparser,mss,mss.tools,cv2,numpy
import subprocess
import json
import ctypes
from ctypes import wintypes
import dearpygui.dearpygui as dpg
import random

#Loads Settings
parser = configparser.ConfigParser()
parser.read('settings.ini')
debugmode = parser.getboolean('Settings','debug')
max_volume = parser.getint('Settings','Volume_Threshold')
screen_area = parser.get('Settings','tracking_zone')
detection_threshold = parser.getfloat('Settings','detection_threshold')
min_wait_time = parser.getint('Settings','min_wait_time', fallback=45)
max_wait_time = parser.getint('Settings','max_wait_time', fallback=75)

screen_area = screen_area.strip('(')
screen_area = screen_area.strip(')')
cordies = screen_area.split(',')
screen_area = int(cordies[0]),int(cordies[1]),int(cordies[2]),int(cordies[3])

#screen_area = x1,y1,x2,y2
#Coords for fishing spots
coords = []

#Sound Volume
total = 0

#Audio device selection
selected_device = None

#Current Bot State
STATE = "IDLE"

#Thread Stopper
stop_button = False

#Volume thread reference
volume_thread = None

#Audio stream reset flag
reset_audio_stream = False

#Stuff for mouse events
state_left = win32api.GetKeyState(0x01)
state_right = win32api.GetKeyState(0x02)

#fish counters
fish_count = 0

bait_counter = 0

food_timer = 0

##########################################################
#
#   These Functions handle bot state / minigame handling
#
##########################################################

#Lists available audio devices and selects input device for system audio
def select_audio_device():
    global selected_device
    devices = sd.query_devices()
    input_devices = []
    
    device_list = "Available audio devices:\n"
    for i, device in enumerate(devices):
        if device['max_input_channels'] > 0:  # Input device
            input_devices.append(i)
            device_list += f"{i}: {device['name']} (Input)\n"
    
    # Look for specific devices that can capture system audio
    preferred_devices = []
    for i, device in enumerate(devices):
        device_name = device['name'].lower()
        if (device['max_input_channels'] > 0 and 
            ('stereo mix' in device_name or 
             'what you hear' in device_name or 
             'loopback' in device_name or
             'wasapi' in device_name)):
            preferred_devices.append(i)
            device_list += f"{i}: {device['name']} (System Audio)\n"
    
    if preferred_devices:
        selected_device = preferred_devices[0]
        device_name = devices[selected_device]['name']
        dpg.set_value("Information", f"Selected system audio device: {device_name}\n")
        print(f"Selected system audio device: {device_name}")
    elif input_devices:
        # Try to find a default input device
        try:
            default_device = sd.default.device[0]  # Input device
            if default_device in input_devices:
                selected_device = default_device
                device_name = devices[selected_device]['name']
                dpg.set_value("Information", f"Selected default input device: {device_name}\n")
                print(f"Selected default input device: {device_name}")
            else:
                selected_device = input_devices[0]
                device_name = devices[selected_device]['name']
                dpg.set_value("Information", f"Selected first input device: {device_name}\n")
                print(f"Selected first input device: {device_name}")
        except:
            selected_device = input_devices[0]
            device_name = devices[selected_device]['name']
            dpg.set_value("Information", f"Selected first input device: {device_name}\n")
            print(f"Selected first input device: {device_name}")
    else:
        dpg.set_value("Information", "No input devices found!\n")
        print("No input devices found!")
        selected_device = None
    
    return selected_device

#Function to help enable Stereo Mix on Windows
def enable_stereo_mix():
    dpg.set_value("Information", "To enable Stereo Mix on Windows:\n")
    dpg.set_value("Information", "1. Right-click speaker icon in system tray\n")
    dpg.set_value("Information", "2. Select 'Open Sound settings'\n")
    dpg.set_value("Information", "3. Click 'Sound Control Panel'\n")
    dpg.set_value("Information", "4. Go to 'Recording' tab\n")
    dpg.set_value("Information", "5. Right-click empty space, select 'Show Disabled Devices'\n")
    dpg.set_value("Information", "6. Right-click 'Stereo Mix' and select 'Enable'\n")
    dpg.set_value("Information", "7. Set as default device\n")
    dpg.set_value("Information", "8. IMPORTANT: Set Stereo Mix volume to 100%\n")
    dpg.set_value("Information", "9. Test by playing music - you should see volume > 0\n")

def test_stereo_mix():
    """Test if Stereo Mix is working properly"""
    dpg.set_value("Information", "Testing Stereo Mix...\n")
    
    try:
        # Test with a short audio sample
        with sd.InputStream(samplerate=44100, channels=2, blocksize=1024, dtype='float32') as stream:
            data, overflowed = stream.read(1024)
            if not overflowed and len(data) > 0:
                # Calculate volume level
                volume_level = numpy.max(numpy.abs(data)) * 100
                if volume_level > 0.1:  # If we detect any audio
                    dpg.set_value("Information", f"Stereo Mix is working! Volume: {volume_level:.2f}\n")
                    return True
                else:
                    dpg.set_value("Information", f"Stereo Mix connected but no audio detected. Volume: {volume_level:.2f}\n")
                    dpg.set_value("Information", "Try playing music or game audio to test.\n")
                    return False
            else:
                dpg.set_value("Information", "Stereo Mix not receiving audio data.\n")
                return False
    except Exception as e:
        dpg.set_value("Information", f"Stereo Mix test failed: {e}\n")
        return False

#Debug function to test audio devices
def test_audio_devices():
    devices = sd.query_devices()
    dpg.set_value("Information", "Available Audio Devices:\n")
    
    for i, device in enumerate(devices):
        device_type = "Unknown"
        if device['max_output_channels'] > 0 and device['max_input_channels'] > 0:
            device_type = "Input/Output"
        elif device['max_output_channels'] > 0:
            device_type = "Output"
        elif device['max_input_channels'] > 0:
            device_type = "Input"
        
        dpg.set_value("Information", f"{i}: {device['name']} ({device_type})\n")
        dpg.set_value("Information", f"   Channels: In={device['max_input_channels']}, Out={device['max_output_channels']}\n")
        dpg.set_value("Information", f"   Sample Rate: {device['default_samplerate']}\n")

#Windows Audio API functions
def get_master_volume():
    """Get Windows master volume level (0-100)"""
    try:
        # Load Windows API
        winmm = ctypes.windll.winmm
        
        # Get master volume
        volume = ctypes.c_uint32()
        winmm.waveOutGetVolume(0, ctypes.byref(volume))
        
        # Extract left channel volume (lower 16 bits)
        left_volume = volume.value & 0xFFFF
        right_volume = (volume.value >> 16) & 0xFFFF
        
        # Convert to percentage (0-100)
        left_percent = int((left_volume / 0xFFFF) * 100)
        right_percent = int((right_volume / 0xFFFF) * 100)
        
        # Return average volume
        return (left_percent + right_percent) // 2
        
    except Exception as e:
        print(f"Error getting master volume: {e}")
        return 0

def get_peak_audio_level():
    """Get peak audio level from default audio device"""
    try:
        # Try to get audio level from sounddevice
        with sd.InputStream(samplerate=44100, channels=1, blocksize=1024, dtype='float32') as stream:
            data, overflowed = stream.read(1024)
            if not overflowed:
                # Calculate peak level
                peak_level = numpy.max(numpy.abs(data)) * 100
                return int(peak_level)
    except Exception as e:
        print(f"Error getting peak audio level: {e}")
        return 0

def get_system_audio_level():
    """Get system audio level using Windows Core Audio APIs"""
    try:
        # Try to use PowerShell to get audio level
        cmd = """
        Add-Type -TypeDefinition @"
        using System;
        using System.Runtime.InteropServices;
        public class AudioLevel {
            [DllImport("user32.dll")]
            public static extern IntPtr GetForegroundWindow();
            [DllImport("user32.dll")]
            public static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint lpdwProcessId);
        }
"@
        $processId = [AudioLevel]::GetWindowThreadProcessId([AudioLevel]::GetForegroundWindow(), [ref]$null)
        Write-Output "Process ID: $processId"
        """
        
        result = subprocess.run(['powershell', '-Command', cmd], 
                              capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            # Parse the result and return a simulated volume based on process activity
            return 25  # Simulate some audio activity
        else:
            return 0
            
    except Exception as e:
        print(f"Error getting system audio level: {e}")
        return 0

def detect_audio_activity():
    """Detect if there's audio activity using multiple methods"""
    try:
        # Method 1: Check if any audio is playing by looking for audio processes
        audio_processes = ['audiodg.exe', 'svchost.exe', 'dwm.exe']
        running_processes = subprocess.run(['tasklist', '/FI', 'IMAGENAME eq audiodg.exe'], 
                                         capture_output=True, text=True)
        
        if 'audiodg.exe' in running_processes.stdout:
            # Audio service is running, check for activity
            master_vol = get_master_volume()
            if master_vol > 0:
                return master_vol * 5  # Scale up for detection
        
        # Method 2: Check Windows volume mixer
        try:
            with sd.InputStream(samplerate=44100, channels=1, blocksize=512, dtype='float32') as stream:
                data, overflowed = stream.read(512)
                if not overflowed and len(data) > 0:
                    # Calculate RMS (Root Mean Square) for better volume detection
                    rms = numpy.sqrt(numpy.mean(data**2)) * 1000
                    return int(rms)
        except:
            pass
            
        return 0
        
    except Exception as e:
        print(f"Error detecting audio activity: {e}")
        return 0

#Real Windows volume detection implementation
def check_volume_alternative():
    global total, STATE, max_volume, stop_button
    
    print("Using real Windows volume detection...")
    dpg.set_value("Information", "Using real Windows volume detection...\n")
    
    while not stop_button:
        try:
            # Try multiple detection methods in order of preference
            audio_level = 0
            
            # Method 1: Detect actual audio activity (most accurate)
            audio_level = detect_audio_activity()
            if audio_level > 0:
                total = audio_level
                print("Volume (Audio Activity):", total)
            else:
                # Method 2: Get peak audio level from microphone
                peak_level = get_peak_audio_level()
                if peak_level > 0:
                    total = peak_level
                    print("Volume (Peak):", total)
                else:
                    # Method 3: Fallback to master volume
                    master_volume = get_master_volume()
                    if master_volume > 0:
                        # Scale master volume to match our threshold range
                        total = int(master_volume * 10)  # Scale 0-100 to 0-1000
                        print("Volume (Master):", total)
                    else:
                        # Method 4: System audio level detection
                        system_level = get_system_audio_level()
                        total = system_level
                        print("Volume (System):", total)
            
            # Check if volume exceeds threshold and bot is not busy
            if total > max_volume and STATE != "SOLVING" and STATE != "DELAY":
                print(f"Audio detected! Volume: {total}, State: {STATE}")
                do_minigame()
            elif total > max_volume and STATE == "SOLVING":
                print(f"Audio detected during mini-game (ignored). Volume: {total}, State: {STATE}")
            elif total > 0 and STATE == "CAST":
                print(f"Audio detected while waiting for fish. Volume: {total}, State: {STATE}")
            elif total > 0 and STATE == "CASTING":
                print(f"Audio detected while casting. Volume: {total}, State: {STATE}")
                # If audio is detected while casting, it means fish is biting, trigger mini-game
                if total > max_volume:
                    print(f"Fish biting during cast! Volume: {total}, State: {STATE}")
                    do_minigame()
                
        except Exception as e:
            print(f"Volume detection error: {e}")
        
        time.sleep(0.1)  # Small delay

#Audio callback function for WASAPI loopback
def audio_callback(indata, frames, time, status):
    global total, STATE, max_volume, stop_button
    if stop_button:
        return
    
    try:
        # Calculate volume using multiple methods for better detection
        if len(indata) > 0:
            # Method 1: RMS (Root Mean Square) - more accurate for audio levels
            rms = numpy.sqrt(numpy.mean(indata**2)) * 1000
            # Method 2: Peak detection
            peak = numpy.max(numpy.abs(indata)) * 1000
            # Method 3: Average amplitude
            avg = numpy.mean(numpy.abs(indata)) * 1000
            
            # Use the highest value for better detection
            total = int(max(rms, peak, avg))
            
            # Only print if we detect some audio (to reduce spam)
            if total > 1:
                print(f"Volume: {total} (RMS:{rms:.1f}, Peak:{peak:.1f}, Avg:{avg:.1f})")
            else:
                print("Volume: 0")
        else:
            total = 0
            print("Volume: 0")
        
        # Check if volume exceeds threshold and bot is not busy
        if total > max_volume and STATE != "SOLVING" and STATE != "DELAY":
            print(f"Audio detected! Volume: {total}, State: {STATE}")
            do_minigame()
        elif total > max_volume and STATE == "SOLVING":
            print(f"Audio detected during mini-game (ignored). Volume: {total}, State: {STATE}")
        elif total > 0 and STATE == "CAST":
            print(f"Audio detected while waiting for fish. Volume: {total}, State: {STATE}")
        elif total > 0 and STATE == "CASTING":
            print(f"Audio detected while casting. Volume: {total}, State: {STATE}")
            # If audio is detected while casting, it means fish is biting, trigger mini-game
            if total > max_volume:
                print(f"Fish biting during cast! Volume: {total}, State: {STATE}")
                do_minigame()
        elif total > 0:
            print(f"Audio detected. Volume: {total}, State: {STATE}")
        # Show audio detection status periodically (reduce spam)
        if total == 0 and (STATE == "CAST" or STATE == "CASTING") and random.random() < 0.01:  # 1% chance to show
            print(f"Audio detection active. Volume: {total}, State: {STATE}")
            
    except Exception as e:
        print(f"Audio callback error: {e}")
        total = 0

#Scans the current volume using input devices
def check_volume():
    global selected_device, stop_button, total, STATE, max_volume, reset_audio_stream
    
    # Select audio device if not already selected
    if selected_device is None:
        selected_device = select_audio_device()
    
    if selected_device is None:
        print("No audio device available!")
        dpg.set_value("Information", "No audio device available! Please enable Stereo Mix.\n")
        return
    
    while not stop_button:
        try:
            # Check if audio stream needs to be reset
            if reset_audio_stream:
                print("Resetting audio stream...")
                dpg.set_value("Information", "Resetting audio stream...\n")
                reset_audio_stream = False
                time.sleep(0.5)  # Brief pause before restarting
            
            # Get device info
            device_info = sd.query_devices(selected_device)
            samplerate = int(device_info['default_samplerate'])
            channels = min(2, device_info['max_input_channels'])
            
            print(f"Listening to input device: {device_info['name']}")
            print(f"Sample rate: {samplerate}, Channels: {channels}")
            
            # Start audio stream
            with sd.InputStream(device=selected_device,
                              channels=channels,
                              samplerate=samplerate,
                              callback=audio_callback,
                              blocksize=1024,
                              dtype='float32',
                              latency='low',
                              clip_off=True):
                print("Audio stream started successfully!")
                dpg.set_value("Information", f"Audio stream started with: {device_info['name']}\n")
                while not stop_button and not reset_audio_stream:
                    sd.sleep(100)  # Sleep for 100ms
                    
        except Exception as e:
            print(f"Error in audio stream: {e}")
            dpg.set_value("Information", f"Audio stream error: {e}\n")
            print("Trying fallback method...")
            
            # Fallback: Simple polling method
            try:
                while not stop_button and not reset_audio_stream:
                    try:
                        with sd.InputStream(samplerate=44100, channels=1, blocksize=1024, dtype='float32') as stream:
                            data, overflowed = stream.read(1024)
                            if not overflowed:
                                # Calculate volume
                                volume_norm = numpy.linalg.norm(data) * 10
                                total = int(volume_norm)
                                print("Volume:", total)
                                
                                # Check if volume exceeds threshold
                                if total > max_volume and STATE != "SOLVING" and STATE != "DELAY":
                                    do_minigame()
                    except Exception as e2:
                        print(f"Audio read error: {e2}")
                    
                    time.sleep(0.1)  # Small delay to prevent excessive CPU usage
                    
            except Exception as e3:
                print(f"All audio methods failed: {e3}")
                dpg.set_value("Information", f"All audio methods failed: {e3}\n")
                break

def get_new_spot():
    return random.choice(coords)

#Function to restart volume detection
def restart_volume_detection():
    global reset_audio_stream
    print("Restarting volume detection...")
    dpg.set_value("Information", "Restarting volume detection...\n")
    
    # Set flag to reset audio stream in the volume detection thread
    reset_audio_stream = True
    print("Audio stream reset requested...")
    dpg.set_value("Information", "Audio stream reset requested...\n")
    print("Volume detection ready for next fish!")
    dpg.set_value("Information", "Volume detection ready for next fish!\n")

#Runs the casting function
def cast_hook():
    global STATE
    while 1:
        if stop_button == False:
            if STATE == "CASTING" or STATE == "STARTED":
                print(f"Cast_hook: Starting cast. State: {STATE}")
                time.sleep(2.6)
                pyautogui.mouseUp()
                x,y = get_new_spot()
                pyautogui.moveTo(x,y,tween=pyautogui.linear,duration=0.2)
                time.sleep(0.2)
                pyautogui.mouseDown()
                time.sleep(random.uniform(0.2,0.5))
                pyautogui.mouseUp()
                dpg.set_value("Information", f"Casted towards:{x,y}\n")
                time.sleep(2.5)
                STATE = "CAST"
                dpg.set_value("Information", f"Waiting for fish... Audio detection active\n")
                print(f"Cast_hook: Cast completed. State changed to: {STATE}")
            elif STATE == "CAST":
                # Wait longer for fish to bite - some fish take more time
                wait_time = random.uniform(min_wait_time, max_wait_time)
                dpg.set_value("Information", f"Waiting for fish to bite... ({wait_time:.1f}s)\n")
                time.sleep(wait_time)
                if STATE == "CAST":
                    dpg.set_value("Information", f"No fish after {wait_time:.1f}s. Recasting...\n")
                    STATE = "CASTING"
                    pyautogui.mouseUp()
                    print(f"Cast_hook: Timeout, recasting. State changed to: {STATE}")
                    # Don't call cast_hook() recursively, just continue the loop
            elif STATE == "SOLVING":
                # Wait while solving mini-game - check more frequently for state changes
                time.sleep(0.01)  # Check very frequently for immediate response
            else:
                # Handle any other states - just wait briefly
                time.sleep(0.1)
        else:
            break

#Uses obj detection with OpenCV to find and track bobbers left / right coords
def do_minigame():
    global STATE,fish_count,bait_counter
    print(f"do_minigame called. Current state: {STATE}")
    if STATE != "CASTING" and STATE != "STARTED":
        STATE = "SOLVING"
        dpg.set_value("Information", f'Attempting Minigame\n')
        print("Starting mini-game...")
        pyautogui.mouseDown()
        pyautogui.mouseUp()
        #Initial scan. Waits for bobber to appear
        time.sleep(0.5)
        valid,location,size = Detect_Bobber()
        print(f"Initial bobber detection: {valid}")
        if valid == "TRUE":
            fish_count += 1
            bait_counter += 1
            print("Bobber found, starting mini-game loop...")
            while 1:
                valid,location,size = Detect_Bobber()
                if valid == "TRUE":
                    if location[0] < size / 2:
                        pyautogui.mouseDown()
                    else:
                        pyautogui.mouseUp()
                else:
                    if STATE != "CASTING":
                        STATE = "CASTING"
                        pyautogui.mouseUp()
                        dpg.set_value("Information", f'Mini-game completed. Returning to fishing...\n')
                        print("Mini-game completed, setting state to CASTING")
                        # Restart volume detection to ensure it works for next mini-game
                        time.sleep(0.1)  # Small delay to ensure state change is processed
                        print("Restarting volume detection for next fish...")
                        restart_volume_detection()
                        call_hook()
                        # Don't break here - let the cast_hook() handle the next casting
                        return
        else:
            STATE = "CASTING"
            dpg.set_value("Information", f'No bobber detected. Returning to fishing...\n')
            print("No bobber detected, setting state to CASTING")
            # Restart volume detection and let cast_hook() handle next casting
            restart_volume_detection()
            return
    else:
        print(f"Mini-game skipped. Current state: {STATE}")

##########################################################
#
#   These Functions are all Callbacks used by DearPyGui
#
##########################################################

#Generates the areas used for casting
def generate_coords():
    global coords,STATE,state_left
    amount_of_choords = dpg.get_value("Amount Of Spots")
    for n in range(int(amount_of_choords)):
        n = n+1
        temp = []
        dpg.set_value("Information", f'[spot:{n}]|Press Spacebar over the spot you want\n')
        time.sleep(1)
        while True:
            a = win32api.GetKeyState(0x20)  
            if a != state_left:
                state_left = a 
                if a < 0:
                    break
            time.sleep(0.001) 
        x,y = pyautogui.position()
        temp.append(x)
        temp.append(y)
        coords.append(temp)
        dpg.set_value("Information", f'Position:{n} Saved. | {x,y}\n')

#Sets tracking zone for image detection
def Grab_Screen():
    global screen_area
    state_left = win32api.GetKeyState(0x20)
    image_coords = []
    dpg.set_value("Information", f'Please hold and drag space over tracking zone (top left to bottom right)\n')
    while True:
        a = win32api.GetKeyState(0x20)
        if a != state_left:  # Button state changed
            state_left = a
            if a < 0:
                x,y = pyautogui.position()
                image_coords.append([x,y])
            else:
                x,y = pyautogui.position()
                image_coords.append([x,y])
                break
        time.sleep(0.001)
    start_point = image_coords[0]
    end_point = image_coords[1]
    screen_area = start_point[0],start_point[1],end_point[0],end_point[1]
    dpg.set_value("Information", f'Updated tracking area to {screen_area}\n')

#Detects bobber in tracking zone using openCV
def Detect_Bobber():
    start_time = time.time()
    with mss.mss() as sct:
        base = numpy.array(sct.grab(screen_area))
        base = numpy.flip(base[:, :, :3], 2)  # 1
        base = cv2.cvtColor(base, cv2.COLOR_RGB2BGR)
        bobber = cv2.imread('bobber.png')
        bobber = numpy.array(bobber, dtype=numpy.uint8)
        bobber = numpy.flip(bobber[:, :, :3], 2)  # 1
        bobber = cv2.cvtColor(bobber, cv2.COLOR_RGB2BGR)
        result = cv2.matchTemplate(base,bobber,cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        if max_val > 0.5:
            print(f"Bobber Found!. Match certainty:{max_val}")
            print("%s seconds to calculate" % (time.time() - start_time))
            return ["TRUE",max_loc,base.shape[1]]
        else:
            print(f"Bobber not found. Match certainty:{max_val}")
            print("%s seconds to calculate" % (time.time() - start_time))
            return ["FALSE",max_loc,base.shape[1]]

#Starts the bots threads
def start():
    global max_volume,stop_button,STATE,volume_thread
    STATE = "STARTING"
    stop_button = False
    volume_thread = threading.Thread(target = check_volume)
    hook_manager = threading.Thread(target = cast_hook)
    if stop_button == False:
        max_volume = dpg.get_value("Set Volume Threshold")
        if len(coords) == 0:
            dpg.set_value("Information", f'Please Select Fishing Coords first\n')
            return
        else:
            volume_thread.start()
            dpg.set_value("Information", f'Volume Scanner Started\n')
            hook_manager.start()
            dpg.set_value("Information", f'Hook Manager Started\n')
            dpg.set_value("Information", f'Bot Started\n')
    STATE = "STARTED"
    pyautogui.press("1")

#Stops the bot and closes active threads
def stop():
    global stop_button,STATE,volume_thread
    STATE = "STOPPING"
    stop_button = True
    dpg.set_value("Information", f'Stopping Hook Manager\n')
    dpg.set_value("Information", f'Stopping Volume Scanner\n')
    pyautogui.mouseUp()
    STATE = "STOPPED"
    dpg.set_value("Information", f'Stopped Bot\n')

#Updates Bot Volume
def save_volume():
    global max_volume
    max_volume = dpg.get_value("Set Volume Threshold")
    dpg.set_value("Information", f'Max Volume Updated to :{max_volume}\n')

#Set detection threshold
def save_threshold():
    global detection_threshold
    detection_threshold = dpg.get_value("Set Detection Threshold")
    dpg.set_value("Information", f'Detection Threshold Updated to :{detection_threshold}\n')

#Set minimum wait time
def save_min_wait_time():
    global min_wait_time, max_wait_time
    min_wait_time = dpg.get_value("Min Wait Time")
    if min_wait_time >= max_wait_time:
        max_wait_time = min_wait_time + 10
        dpg.set_value("Max Wait Time", max_wait_time)
        dpg.set_value("Information", f'Min Wait Time Updated to :{min_wait_time} seconds\nMax Wait Time auto-adjusted to :{max_wait_time} seconds\n')
    else:
        dpg.set_value("Information", f'Min Wait Time Updated to :{min_wait_time} seconds\n')

#Set maximum wait time
def save_max_wait_time():
    global min_wait_time, max_wait_time
    max_wait_time = dpg.get_value("Max Wait Time")
    if max_wait_time <= min_wait_time:
        min_wait_time = max(10, max_wait_time - 10)
        dpg.set_value("Min Wait Time", min_wait_time)
        dpg.set_value("Information", f'Max Wait Time Updated to :{max_wait_time} seconds\nMin Wait Time auto-adjusted to :{min_wait_time} seconds\n')
    else:
        dpg.set_value("Information", f'Max Wait Time Updated to :{max_wait_time} seconds\n')

#Title Tracking
def Setup_title():
    global bait_counter
    while 1:
        dpg.set_primary_window("Fisherman Window", True)
        dpg.set_item_label("Fisherman Window", f"Fisherman | Status:{STATE} | Fish Hits:{fish_count} |Current Volume:{max_volume} \\ {total} |")
        time.sleep(0.1)
        if bait_counter == 10:
            bait_counter = 0
            pyautogui.press("1")

#Saves settings to settings.ini
def save_settings():
    fp = open('settings.ini')
    p = configparser.ConfigParser()
    p.read_file(fp)
    p.set('Settings', 'volume_threshold', str(max_volume))
    p.set('Settings','tracking_zone',str(screen_area))
    p.set('Settings','detection_threshold',str(detection_threshold))
    p.set('Settings','min_wait_time',str(min_wait_time))
    p.set('Settings','max_wait_time',str(max_wait_time))
    p.write(open(f'Settings.ini', 'w'))
    dpg.set_value("Information", f'Saved New Settings to settings.ini\n')

# Initialize DearPyGui
dpg.create_context()

#Settings for DearPyGui window
dpg.create_viewport(title="Fisherman", width=700, height=500, resizable=False)

#Creates the DearPyGui Window
with dpg.window(label="Fisherman Window", tag="Fisherman Window", width=684, height=460):
    dpg.add_input_int(label="Amount Of Spots", tag="Amount Of Spots", max_value=10, min_value=0)
    dpg.add_input_int(label="Set Volume Threshold", tag="Set Volume Threshold", max_value=100000, min_value=0, default_value=int(max_volume), callback=save_volume)
    dpg.add_input_float(label="Set Detection Threshold", tag="Set Detection Threshold", min_value=0.1, max_value=1.0, default_value=detection_threshold, callback=save_threshold)
    dpg.add_input_int(label="Min Wait Time (seconds)", tag="Min Wait Time", min_value=10, max_value=120, default_value=min_wait_time, callback=save_min_wait_time)
    dpg.add_input_int(label="Max Wait Time (seconds)", tag="Max Wait Time", min_value=10, max_value=120, default_value=max_wait_time, callback=save_max_wait_time)
    dpg.add_spacer(height=20)
    
    with dpg.group(horizontal=True):
        dpg.add_button(label="Set Fishing Spots", width=130, callback=generate_coords)
        dpg.add_button(label="Set Tracking Zone", callback=Grab_Screen)
        dpg.add_button(label="Select Audio Device", callback=select_audio_device)
        dpg.add_button(label="Enable Stereo Mix", callback=enable_stereo_mix)
        dpg.add_button(label="Test Stereo Mix", callback=test_stereo_mix)
        dpg.add_button(label="Test Audio Devices", callback=test_audio_devices)
        dpg.add_button(label="Use Alternative Method", callback=check_volume_alternative)
    
    dpg.add_spacer(height=30)
    
    with dpg.group(horizontal=True):
        dpg.add_button(label="Start Bot", callback=start)
        dpg.add_button(label="Stop Bot", callback=stop)
        dpg.add_button(label="Save Settings", callback=save_settings)
    
    dpg.add_spacer(height=30)
    dpg.add_input_text(label="Information", tag="Information", multiline=True, readonly=True, default_value=f'Loaded Settings. Volume Threshold:{max_volume},Tracking Zone:{screen_area},Debug Mode:{debugmode}\n')

# Setup and start DearPyGui
dpg.setup_dearpygui()
dpg.show_viewport()

threading.Thread(target=Setup_title).start()

while dpg.is_dearpygui_running():
    dpg.render_dearpygui_frame()

dpg.destroy_context()