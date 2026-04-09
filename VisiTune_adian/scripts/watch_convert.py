import os
import time
from pydub import AudioSegment

SOURCE_DIR = "input"
DEST_DIR = "converted"
CHECK_INTERVAL = 2  # seconds between scans

SUPPORTED_INPUTS = [".mp3", ".wav", ".aac", ".flac", ".ogg", ".m4a"]

# Create output directory if needed
os.makedirs(DEST_DIR, exist_ok=True)

# Keep track of files already processed
processed = set(os.listdir(DEST_DIR))

def convert_to_wav(input_path, output_path):
    try:
        audio = AudioSegment.from_file(input_path)
        audio = audio.set_frame_rate(44100).set_sample_width(2).set_channels(1)
        audio.export(output_path, format="wav")
        print(f"[OK] Converted: {input_path} → {output_path}")
    except Exception as e:
        print(f"[ERROR] Failed to convert {input_path}: {e}")

def main():
    print("Watching for new audio files...\n")

    while True:
        for filename in os.listdir(SOURCE_DIR):
            if filename.lower().endswith(tuple(SUPPORTED_INPUTS)):
                input_path = os.path.join(SOURCE_DIR, filename)
                output_name = os.path.splitext(filename)[0] + ".wav"
                output_path = os.path.join(DEST_DIR, output_name)

                # Skip if already converted
                if output_name in processed:
                    continue

                convert_to_wav(input_path, output_path)
                processed.add(output_name)

        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    main()
