import os
import time
from pydub import AudioSegment
from PIL import Image

SOURCE_DIR = "input"
DEST_DIR = "converted"
CHECK_INTERVAL = 2  # seconds

SUPPORTED_AUDIO = [".mp3", ".wav", ".aac", ".flac", ".ogg", ".m4a"]
SUPPORTED_IMAGES = [".jpg", ".jpeg", ".png", ".bmp", ".gif", ".webp", ".tiff"]

os.makedirs(SOURCE_DIR, exist_ok=True)
os.makedirs(DEST_DIR, exist_ok=True)

# Track files we've processed internally
processed_files = set()

# Track waiting files (files waiting to be paired)
waiting_audio = []
waiting_images = []

# Determine next pair index
def get_next_index():
    existing = [f for f in os.listdir(DEST_DIR) if f.endswith(".wav")]
    nums = []
    for f in existing:
        try:
            n = int(os.path.splitext(f)[0])
            nums.append(n)
        except:
            pass
    return max(nums, default=0) + 1


# ---------------------
# Conversion Functions
# ---------------------

def convert_audio(input_path, output_path):
    try:
        audio = AudioSegment.from_file(input_path)
        audio = audio.set_frame_rate(24000).set_sample_width(2).set_channels(1)
        audio.export(output_path, format="wav")
        print(f"[AUDIO OK] {input_path} → {output_path}")
    except Exception as e:
        print(f"[AUDIO ERROR] {input_path}: {e}")

def convert_image(input_path, output_path):
    try:
        img = Image.open(input_path)
        img = img.convert("RGB")  # ensure RGB
        img.save(output_path, format="PNG")
        print(f"[IMAGE OK] {input_path} → {output_path}")
    except Exception as e:
        print(f"[IMAGE ERROR] {input_path}: {e}")


# ---------------------
# Main Loop
# ---------------------

def main():
    print("Watching for audio/image pairs...\n")

    global waiting_audio, waiting_images

    while True:
        for filename in os.listdir(SOURCE_DIR):

            if filename in processed_files:
                continue

            input_path = os.path.join(SOURCE_DIR, filename)
            name, ext = os.path.splitext(filename.lower())

            # -------------------
            # Detect Audio Files
            # -------------------
            if ext in SUPPORTED_AUDIO:
                waiting_audio.append(input_path)
                processed_files.add(filename)
                print(f"[DETECTED AUDIO] {input_path}")

            # -------------------
            # Detect Image Files
            # -------------------
            elif ext in SUPPORTED_IMAGES:
                waiting_images.append(input_path)
                processed_files.add(filename)
                print(f"[DETECTED IMAGE] {input_path}")

        # Pair logic: convert only when we have BOTH
        while waiting_audio and waiting_images:
            audio_file = waiting_audio.pop(0)
            image_file = waiting_images.pop(0)

            pair_index = get_next_index()
            audio_out = os.path.join(DEST_DIR, f"{pair_index}.wav")
            image_out = os.path.join(DEST_DIR, f"{pair_index}.png")

            convert_audio(audio_file, audio_out)
            convert_image(image_file, image_out)

        time.sleep(CHECK_INTERVAL)


if __name__ == "__main__":
    main()
