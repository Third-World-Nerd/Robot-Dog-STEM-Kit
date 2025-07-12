from flask import Flask, request
import os
import time
import wave
import threading
import traceback
from werkzeug.exceptions import ClientDisconnected

app = Flask(__name__)
UPLOAD_FOLDER = 'silence_audio_uploads'

# Create folder if it doesn't exist
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# Audio config (must match ESP32)
SAMPLE_RATE = 16000
BITS_PER_SAMPLE = 16
CHANNELS = 1
BYTES_PER_SAMPLE = BITS_PER_SAMPLE // 8
EXPECTED_DURATION_SEC = 10
EXPECTED_BYTES = SAMPLE_RATE * BYTES_PER_SAMPLE * EXPECTED_DURATION_SEC

# Thread-safe file writing
file_lock = threading.Lock()

def save_audio_with_wav_header(raw_data, filename):
    """Save raw PCM data as WAV"""
    filepath = os.path.join(UPLOAD_FOLDER, filename)

    with wave.open(filepath, 'wb') as wav_file:
        wav_file.setnchannels(CHANNELS)
        wav_file.setsampwidth(BYTES_PER_SAMPLE)
        wav_file.setframerate(SAMPLE_RATE)
        wav_file.writeframes(raw_data)

    return filepath

def clean_old_files(max_files=20):
    """Delete oldest files, keeping only `max_files`"""
    try:
        with file_lock:
            files = [f for f in os.listdir(UPLOAD_FOLDER) if f.endswith('.wav')]
            files.sort(key=lambda f: os.path.getctime(os.path.join(UPLOAD_FOLDER, f)))
            while len(files) > max_files:
                old_file = files.pop(0)
                try:
                    os.remove(os.path.join(UPLOAD_FOLDER, old_file))
                    print(f"[Cleanup] Deleted: {old_file}")
                except Exception as e:
                    print(f"[Cleanup Error] {old_file}: {e}")
    except Exception:
        print("[Cleanup Error]")
        traceback.print_exc()

def get_raw_data_safe(req, max_size: int, timeout=15):
    """Read data from the request stream with a timeout."""
    raw_data = b''
    start_time = time.time()
    while len(raw_data) < max_size:
        # Check timeout
        if time.time() - start_time > timeout:
            print(f"⚠️ Timeout reached. Read {len(raw_data)} bytes, expected {max_size}")
            break
            
        # Calculate remaining bytes and chunk size to read
        remaining = max_size - len(raw_data)
        to_read = min(4096, remaining)
        try:
            chunk = req.stream.read(to_read)
            if not chunk:
                # No data received, break to avoid infinite loop
                break
            raw_data += chunk
        except ClientDisconnected:
            print("Client disconnected during read")
            break
        except Exception as e:
            print(f"Unexpected error during read: {e}")
            traceback.print_exc()
            break
    return raw_data

def save_audio_background(raw_data):
    """Background task to save audio and clean up old files."""
    try:
        # Create safe filename
        timestamp = int(time.time() * 1000)
        filename = f"audio_{timestamp}.wav"
        with file_lock:
            save_audio_with_wav_header(raw_data, filename)
        print(f"Audio saved: {filename}")

        # Clean old files
        # -----()
    except Exception as e:
        print(f"Error in background task: {e}")
        traceback.print_exc()

@app.route('/upload', methods=['POST'])
def upload_file():
    try:
        raw_data = get_raw_data_safe(request, EXPECTED_BYTES, timeout=15)
        size = len(raw_data)
        print(f"[{time.strftime('%H:%M:%S')}] Upload received: {size} bytes")

        if size < 100:
            return 'Invalid audio data (too small)', 400

        # Check if data is expected size (optional)
        if size != EXPECTED_BYTES:
            print(f"⚠️ Warning: Unexpected size (expected {EXPECTED_BYTES}, got {size})")

        # Save in background to release the request
        threading.Thread(target=save_audio_background, args=(raw_data,)).start()

        return "Audio upload processing started.", 202

    except Exception as e:
        print(f"[Upload Error] {e}")
        traceback.print_exc()
        return "Server Error", 500

if __name__ == '__main__':
    print("🎙️ Audio server listening on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000)