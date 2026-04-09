import os
import sys
import struct
import wave
import time
import threading
import shutil

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from queue import Queue, Empty

import serial
from serial.tools import list_ports
from PIL import Image
import numpy as np
from pydub import AudioSegment  # <-- NEW: for audio conversion

# ---------- Auto-convert watcher config ----------

SOURCE_DIR = "input"
DEST_DIR = "converted"
CHECK_INTERVAL = 2  # seconds

SUPPORTED_AUDIO = [".mp3", ".wav", ".aac", ".flac", ".ogg", ".m4a"]
SUPPORTED_IMAGES = [".jpg", ".jpeg", ".png", ".bmp", ".gif", ".webp", ".tiff"]

# ---------- Debug / logging control ----------

VERBOSE_TX = False   # set True if you want to see every packet
VERBOSE_RX = False   # optional: for RX spam too
VERBOSE_LOG = False  # NEW: master switch for debug/info prints



os.makedirs(SOURCE_DIR, exist_ok=True)
os.makedirs(DEST_DIR, exist_ok=True)

# Track files we've processed internally
processed_files = set()

# Track waiting files (files waiting to be paired)
waiting_audio: List[str] = []
waiting_images: List[str] = []

# ---------- Track model ----------

PacketEntry = Tuple[int, int, bytes, str]  # (seq, flags, payload, label)


@dataclass
class Track:
    title: str
    artist: str
    album: str
    image_path: str
    wav_path: str
    image_packets: List[PacketEntry] = field(default_factory=list)
    audio_packets: List[PacketEntry] = field(default_factory=list)
    audio_index: int = 0
    finished: bool = False

def log(msg: str):
    """Debug/info logging controlled by VERBOSE_LOG."""
    if VERBOSE_LOG:
        print(msg)


def build_converted_from_input_folders():
    """
    Scan input/track*/ folders, convert their audio/image to converted/N.*,
    save metadata to converted/N.txt, and return a list of Tracks ready to use in jukebox.
    """
    tracks: List[Track] = []

    if not os.path.isdir(SOURCE_DIR):
        return tracks

    # Get all track folders and sort them numerically (track1, track2, ..., track10)
    track_folders = []
    for entry in os.listdir(SOURCE_DIR):
        track_dir = os.path.join(SOURCE_DIR, entry)
        if os.path.isdir(track_dir) and entry.startswith("track"):
            try:
                # Extract number from "trackN" folder
                track_num = int(entry[5:])  # Skip "track" prefix
                track_folders.append((track_num, track_dir))
            except ValueError:
                # Skip folders that don't have a valid number
                continue
    
    # Sort by track number
    track_folders.sort(key=lambda x: x[0])
    
    for track_num, track_dir in track_folders:
        # Find audio, image, metadata inside this folder
        audio_file = None
        image_file = None
        meta_file = None

        for fname in os.listdir(track_dir):
            fpath = os.path.join(track_dir, fname)
            if not os.path.isfile(fpath):
                continue
            name, ext = os.path.splitext(fname)
            ext = ext.lower()

            if ext in SUPPORTED_AUDIO and audio_file is None:
                audio_file = fpath
            elif ext in SUPPORTED_IMAGES and image_file is None:
                image_file = fpath
            elif ext in (".txt", ".csv") and meta_file is None:
                meta_file = fpath

        if not audio_file or not image_file:
            log(f"[INPUT] Skipping '{track_dir}' (need at least 1 audio + 1 image).")
            continue

        # Parse metadata
        if meta_file:
            title, artist, album = parse_metadata_file(meta_file)
        else:
            base = os.path.splitext(os.path.basename(audio_file))[0]
            title = base
            artist = "unknown"
            album = "unknown"

        pair_index = get_next_index()
        audio_out = os.path.join(DEST_DIR, f"{pair_index}.wav")
        image_out = os.path.join(DEST_DIR, f"{pair_index}.png")
        meta_out = os.path.join(DEST_DIR, f"{pair_index}.txt")

        log(f"[INPUT] Converting folder '{track_dir}' -> index {pair_index}")
        log(f"        AUDIO: {audio_file} -> {audio_out}")
        log(f"        IMAGE: {image_file} -> {image_out}")
        log(f"        META : -> {meta_out}")

        convert_audio(audio_file, audio_out)
        convert_image(image_file, image_out)
        
        # Save metadata to converted folder
        with open(meta_out, "w", encoding="utf-8") as f:
            f.write(f"{title}\n{artist}\n{album}")

        t = Track(
            title=title,
            artist=artist,
            album=album,
            image_path=image_out,
            wav_path=audio_out,
        )
        prepare_track(t, cover_width=320, cover_height=320)
        tracks.append(t)

    return tracks

def parse_metadata_file(meta_path: str):
    title = None
    artist = None
    album = None

    name, ext = os.path.splitext(meta_path)
    ext = ext.lower()

    if ext == ".txt":
        with open(meta_path, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f.readlines()]
        if len(lines) >= 1: title  = lines[0]
        if len(lines) >= 2: artist = lines[1]
        if len(lines) >= 3: album  = lines[2]

    elif ext == ".csv":
        import csv
        with open(meta_path, "r", encoding="utf-8") as f:
            reader = list(csv.reader(f))
        if len(reader) >= 2:
            row = reader[1]
            if len(row) >= 1: title  = row[0].strip()
            if len(row) >= 2: artist = row[1].strip()
            if len(row) >= 3: album  = row[2].strip()

    return (
        title  or "unknown title",
        artist or "unknown artist",
        album  or "unknown album",
    )






def get_next_index() -> int:
    """
    Determine next pair index based on existing N.wav files in DEST_DIR.
    """
    existing = [f for f in os.listdir(DEST_DIR) if f.lower().endswith(".wav")]
    nums = []
    for f in existing:
        try:
            # Extract number from filename like "1.wav"
            n = int(os.path.splitext(f)[0])
            nums.append(n)
        except ValueError:
            continue  # Skip files that don't match the pattern
    return max(nums, default=0) + 1


def convert_audio(input_path: str, output_path: str):
    try:
        # Load with pydub
        audio = AudioSegment.from_file(input_path)

        # Force to: mono, 16-bit PCM, 24 kHz
        audio = audio.set_channels(1)
        audio = audio.set_sample_width(2)      # 16-bit
        audio = audio.set_frame_rate(24000)    # 24 kHz

        # Export as WAV
        audio.export(output_path, format="wav")

        log(f"[AUDIO OK] {input_path} → {output_path}")
    except Exception as e:
        log(f"[AUDIO ERROR] {input_path}: {e}")



def convert_image(input_path: str, output_path: str):
    try:
        # Open the image
        img = Image.open(input_path)
        img = img.convert("RGB")  # ensure RGB
        
        # Get original dimensions
        width, height = img.size
        
        # Calculate the center square crop
        # Use the smaller dimension as the square size
        crop_size = min(width, height)
        
        # Calculate crop coordinates for center
        left = (width - crop_size) // 2
        top = (height - crop_size) // 2
        right = left + crop_size
        bottom = top + crop_size
        
        # Crop to center square
        img_cropped = img.crop((left, top, right, bottom))
        
        # Resize to 320x320
        img_resized = img_cropped.resize((320, 320), Image.Resampling.LANCZOS)
        
        # Save as PNG
        img_resized.save(output_path, format="PNG")
        log(f"[IMAGE OK] {input_path} → {output_path} (center cropped to 320x320)")
    except Exception as e:
        log(f"[IMAGE ERROR] {input_path}: {e}")


def conversion_watcher_loop():
    """
    Background thread that watches SOURCE_DIR for new files.
    When it sees at least one audio and one image waiting,
    it converts them into numbered .wav/.png pairs in DEST_DIR with metadata.
    """
    log(f"[WATCHER] Watching '{SOURCE_DIR}' for audio/image pairs...")
    global waiting_audio, waiting_images

    while True:
        try:
            for filename in os.listdir(SOURCE_DIR):
                if filename in processed_files:
                    continue

                input_path = os.path.join(SOURCE_DIR, filename)
                name, ext = os.path.splitext(filename)
                ext = ext.lower()

                if ext in SUPPORTED_AUDIO:
                    waiting_audio.append(input_path)
                    processed_files.add(filename)
                    log(f"[WATCHER] Detected AUDIO: {input_path}")

                elif ext in SUPPORTED_IMAGES:
                    waiting_images.append(input_path)
                    processed_files.add(filename)
                    log(f"[WATCHER] Detected IMAGE: {input_path}")

            # Pair logic: convert only when we have BOTH
            while waiting_audio and waiting_images:
                audio_file = waiting_audio.pop(0)
                image_file = waiting_images.pop(0)

                pair_index = get_next_index()
                audio_out = os.path.join(DEST_DIR, f"{pair_index}.wav")
                image_out = os.path.join(DEST_DIR, f"{pair_index}.png")
                meta_out = os.path.join(DEST_DIR, f"{pair_index}.txt")

                log(f"[WATCHER] Converting pair #{pair_index}:")
                log(f"           AUDIO: {audio_file} -> {audio_out}")
                log(f"           IMAGE: {image_file} -> {image_out}")

                convert_audio(audio_file, audio_out)
                convert_image(image_file, image_out)
                
                # Create default metadata
                audio_name = os.path.splitext(os.path.basename(audio_file))[0]
                title = audio_name
                artist = "unknown"
                album = "unknown"
                
                with open(meta_out, "w", encoding="utf-8") as f:
                    f.write(f"{title}\n{artist}\n{album}")
                
                log(f"[WATCHER] Created metadata: {meta_out}")

            time.sleep(CHECK_INTERVAL)
        except Exception as e:
            # Don't kill the whole program if the watcher trips
            log(f"[WATCHER] Error in watcher loop: {e}")
            time.sleep(CHECK_INTERVAL)


# ---------- Protocol constants (mirror packets.h) ----------

PACKET_SOF0    = 0xA5
PACKET_SOF1    = 0x5A
PACKET_VERSION = 0x01

PKT_DATA_IMAGE = 0x01  # must match your packets.h
PKT_DATA_AUDIO = 0x02  # must match your packets.h (PKT_DATA_CMD on MCU is 0x03)
PKT_DATA_CTRL  = 0x03
PKT_DATA_ACK   = 0x04
DATA_TYPE_TEXT = 0x05

CTRL_CMD_AUDIO_STOP = 0x01

TEXT_FIELD_TITLE  = 0x01
TEXT_FIELD_ARTIST = 0x02
TEXT_FIELD_ALBUM  = 0x03
TEXT_FIELD_OTHER  = 0x04

PKT_FLAG_LAST  = 1 << 0

MAX_SIZE       = 0xFFFF
OVERHEAD       = 11                     # 2 SOF + 7 header + 2 CRC
MAX_DATA_SIZE  = MAX_SIZE - OVERHEAD    # max payload bytes

# IMPORTANT: STM32 side uses AUD_MAX_PAYLOAD_BYTES = 4096.
# To match that, we limit audio payloads on PC to 4096 bytes.
AUDIO_MAX_PAYLOAD = min(MAX_DATA_SIZE, 4096)

CRC_POLY = 0x1021
CRC_INIT = 0xFFFF

# ---------- Threading primitives ----------

header_ack_event = threading.Event()   # 'H'
ok_ack_event     = threading.Event()   # 'S'
audio_req_event  = threading.Event()   # 'A'
error_event      = threading.Event()   # 'E'

TX_LOCK = threading.Lock()
rx_thread_running = False

# Button events from MCU (0xF0 + 'P'/'N'/'B')
button_event_queue: Queue[str] = Queue()

# Text UI commands from stdin (user typing)
command_queue: Queue[str] = Queue()


def clear_screen():
    # Works for macOS/Linux and Windows
    os.system('cls' if os.name == 'nt' else 'clear')


def print_jukebox_ui(current_track: Optional[Track],
                     queue_tracks: List[Track],
                     history: List[Track],
                     player_state: str):
    clear_screen()
    print("====== VisiTune Jukebox ======\n")
    print(f"State: {player_state}")
    if current_track:
        print(f"Now Playing: {current_track.title} – {current_track.artist} ({current_track.album})")
    else:
        print("Now Playing: [none]")

    print("\nNext up:")
    if queue_tracks:
        for i, t in enumerate(queue_tracks, start=1):
            print(f"  {i}) {t.title} – {t.artist}")
    else:
        print("  [empty]")

    print("\nHistory (most recent last):")
    if history:
        for t in history:
            print(f"  - {t.title} – {t.artist}")
    else:
        print("  [none]")

    print("\nCommands:")
    print("  n        -> skip to next track")
    print("  b        -> go to previous track")
    print("  space    -> play/pause")
    print("  d <idx>  -> delete item #idx from upcoming queue")
    print("  m i j    -> move item i to position j in upcoming queue")
    print("  a        -> add a new song to the queue")
    print("  e        -> edit song metadata")  # NEW: Edit metadata command
    print("  p        -> reprint this UI")
    print("  q        -> exit jukebox and return to menu")
    print("\n(Type commands at the prompt; playback keeps going.)\n")


def parse_ui_command(cmd: str,
                     current_track: Optional[Track],
                     queue_tracks: List[Track],
                     history: List[Track],
                     player_state: str):
    """
    Return a tuple: (action, args)
    where action is one of:
      "next", "prev", "toggle", "delete", "move", "add", "edit", "print", "quit", None
    """
    cmd = cmd.strip()
    if not cmd:
        return None, None

    # single-char commands
    if cmd == "n":
        return "next", None
    if cmd == "b":
        return "prev", None
    if cmd == "p":
        return "print", None
    if cmd == "q":
        return "quit", None
    if cmd == " ":
        return "toggle", None
    if cmd == "a":
        return "add", None  # Add song command
    if cmd == "e":
        return "edit", None  # NEW: Edit metadata command

    parts = cmd.split()
    if parts[0] == "d" and len(parts) == 2:
        try:
            idx = int(parts[1])
            return "delete", idx
        except ValueError:
            print(f"[UI] Invalid index '{parts[1]}' for delete.")
            return None, None

    if parts[0] == "m" and len(parts) == 3:
        try:
            src = int(parts[1])
            dst = int(parts[2])
            return "move", (src, dst)
        except ValueError:
            print(f"[UI] Invalid indices '{parts[1]}', '{parts[2]}' for move.")
            return None, None

    print(f"[UI] Unknown command: {cmd!r}")
    return None, None


# Add this new function to edit track metadata
def edit_track_metadata():
    """
    Allows user to edit metadata for any track in the converted folder.
    """
    print("\n[EDIT] Available tracks to edit:")
    print("-" * 50)
    
    # Scan for available tracks
    available_tracks = scan_available_tracks()
    
    if not available_tracks:
        print("No tracks found in the 'converted/' folder.")
        return
    
    # Display all tracks with their current metadata
    for i, (idx, track) in enumerate(available_tracks, start=1):
        print(f"  {i}) Track #{idx}: {track.title} - {track.artist} ({track.album})")
    
    print("\nEnter the number of the track to edit (or 0 to cancel):")
    
    try:
        choice = input("> ").strip()
        if choice == "0":
            print("[EDIT] Cancelled.")
            return
        
        if not choice.isdigit():
            print("Please enter a number.")
            return
        
        choice_num = int(choice)
        if not (1 <= choice_num <= len(available_tracks)):
            print(f"Please enter a number between 1 and {len(available_tracks)}.")
            return
        
        selected_idx, selected_track = available_tracks[choice_num - 1]
        
        print(f"\n[EDIT] Editing Track #{selected_idx}:")
        print(f"  Current title:  {selected_track.title}")
        print(f"  Current artist: {selected_track.artist}")
        print(f"  Current album:  {selected_track.album}")
        print("\nEnter new metadata (press Enter to keep current value):")
        
        # Prompt for new values with current as default
        new_title = input(f"  Title  [{selected_track.title}]: ").strip()
        new_artist = input(f"  Artist [{selected_track.artist}]: ").strip()
        new_album = input(f"  Album  [{selected_track.album}]: ").strip()
        
        # Use new value if provided, otherwise keep current
        if new_title:
            selected_track.title = new_title
        if new_artist:
            selected_track.artist = new_artist
        if new_album:
            selected_track.album = new_album
        
        # Update the metadata file
        metadata_path = os.path.splitext(selected_track.wav_path)[0] + ".txt"
        
        try:
            with open(metadata_path, "w", encoding="utf-8") as f:
                f.write(f"{selected_track.title}\n{selected_track.artist}\n{selected_track.album}")
            
            print(f"\n[EDIT] ✓ Updated metadata for Track #{selected_idx}")
            print(f"       Title:  {selected_track.title}")
            print(f"       Artist: {selected_track.artist}")
            print(f"       Album:  {selected_track.album}")
            
            # Also update the image packets with the new title/artist/album
            # This is optional but ensures the display shows updated info
            prepare_track(selected_track, cover_width=320, cover_height=320)
            
        except Exception as e:
            print(f"[EDIT] Error saving metadata: {e}")
            
    except (ValueError, KeyboardInterrupt):
        print("[EDIT] Cancelled.")


# Add this non-blocking version for use in jukebox mode
def edit_track_metadata_async(edit_track_queue):
    """
    Non-blocking function to edit track metadata.
    Runs in a separate thread to avoid blocking audio processing.
    """
    print("\n[EDIT] Available tracks to edit:")
    print("-" * 50)
    
    # Scan for available tracks
    available_tracks = scan_available_tracks()
    
    if not available_tracks:
        print("No tracks found in the 'converted/' folder.")
        return
    
    # Display all tracks with their current metadata
    for i, (idx, track) in enumerate(available_tracks, start=1):
        print(f"  {i}) Track #{idx}: {track.title} - {track.artist} ({track.album})")
    
    print("\nEnter the number of the track to edit (or 0 to cancel):")
    print("(You can continue to use other commands while editing)")
    
    # Use a simple timeout-based input check so we don't block
    import select
    import sys
    
    timeout_count = 0
    max_timeout = 300  # 30 seconds timeout
    
    while timeout_count < max_timeout:
        # Check if input is available without blocking
        if select.select([sys.stdin], [], [], 0.1)[0]:
            choice = sys.stdin.readline().strip()
            
            if choice == "0":
                print("[EDIT] Cancelled.")
                return
            
            if not choice.isdigit():
                print("Please enter a number.")
                # Reset timeout since we got input
                timeout_count = 0
                continue
            
            choice_num = int(choice)
            if not (1 <= choice_num <= len(available_tracks)):
                print(f"Please enter a number between 1 and {len(available_tracks)}.")
                # Reset timeout since we got input
                timeout_count = 0
                continue
            
            selected_idx, selected_track = available_tracks[choice_num - 1]
            
            print(f"\n[EDIT] Editing Track #{selected_idx}:")
            print(f"  Current title:  {selected_track.title}")
            print(f"  Current artist: {selected_track.artist}")
            print(f"  Current album:  {selected_track.album}")
            print("\nEnter new metadata (press Enter to keep current value):")
            
            # We need to get input for each field without blocking
            # This is tricky in non-blocking mode, so we'll use a simpler approach
            # Just get all input at once with a prompt
            
            print("Enter new title (or press Enter to keep current):")
            new_title = None
            for _ in range(150):  # Wait up to 3 seconds for input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    new_title = sys.stdin.readline().strip()
                    break
            
            if new_title is None:
                print("[EDIT] Input timeout. Keeping current title.")
                new_title = selected_track.title
            
            print("Enter new artist (or press Enter to keep current):")
            new_artist = None
            for _ in range(150):  # Wait up to 3 seconds for input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    new_artist = sys.stdin.readline().strip()
                    break
            
            if new_artist is None:
                print("[EDIT] Input timeout. Keeping current artist.")
                new_artist = selected_track.artist
            
            print("Enter new album (or press Enter to keep current):")
            new_album = None
            for _ in range(150):  # Wait up to 3 seconds for input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    new_album = sys.stdin.readline().strip()
                    break
            
            if new_album is None:
                print("[EDIT] Input timeout. Keeping current album.")
                new_album = selected_track.album
            
            # Update the track
            if new_title:
                selected_track.title = new_title
            if new_artist:
                selected_track.artist = new_artist
            if new_album:
                selected_track.album = new_album
            
            # Update the metadata file
            metadata_path = os.path.splitext(selected_track.wav_path)[0] + ".txt"
            
            try:
                with open(metadata_path, "w", encoding="utf-8") as f:
                    f.write(f"{selected_track.title}\n{selected_track.artist}\n{selected_track.album}")
                
                print(f"\n[EDIT] ✓ Updated metadata for Track #{selected_idx}")
                print(f"       Title:  {selected_track.title}")
                print(f"       Artist: {selected_track.artist}")
                print(f"       Album:  {selected_track.album}")
                
                # Prepare track with new metadata
                prepare_track(selected_track, cover_width=320, cover_height=320)
                
                # Put the updated track in the queue for the main thread
                # This allows the main thread to refresh any display of this track
                edit_track_queue.put((selected_idx, selected_track))
                
            except Exception as e:
                print(f"[EDIT] Error saving metadata: {e}")
            
            return
        
        else:
            timeout_count += 1
            
            # Periodically show we're still waiting
            if timeout_count % 10 == 0:  # Every 1 second
                print(".", end="", flush=True)
    
    print("\n[EDIT] Input timeout. Cancelling metadata edit.")
    return


def scan_available_tracks():
    """
    Scan the converted directory for available tracks and return a list.
    This function now properly reads metadata from the .txt files in input/track*/ folders.
    """
    available_tracks = []
    
    # First, scan input/track*/ folders for metadata
    if not os.path.isdir(SOURCE_DIR):
        return available_tracks
    
    track_folders = []
    for entry in os.listdir(SOURCE_DIR):
        track_dir = os.path.join(SOURCE_DIR, entry)
        if os.path.isdir(track_dir) and entry.startswith("track"):
            try:
                # Extract number from "trackN" folder
                track_num = int(entry[5:])  # Skip "track" prefix
                track_folders.append((track_num, track_dir))
            except ValueError:
                continue
    
    # Sort by track number
    track_folders.sort(key=lambda x: x[0])
    
    for track_num, track_dir in track_folders:
        # Find audio, image, metadata inside this folder
        audio_file = None
        image_file = None
        meta_file = None
        
        for fname in os.listdir(track_dir):
            fpath = os.path.join(track_dir, fname)
            if not os.path.isfile(fpath):
                continue
            name, ext = os.path.splitext(fname)
            ext = ext.lower()
            
            if ext in SUPPORTED_AUDIO and audio_file is None:
                audio_file = fpath
            elif ext in SUPPORTED_IMAGES and image_file is None:
                image_file = fpath
            elif ext in (".txt", ".csv") and meta_file is None:
                meta_file = fpath
        
        if not audio_file or not image_file:
            continue
        
        # Parse metadata
        if meta_file:
            title, artist, album = parse_metadata_file(meta_file)
        else:
            base = os.path.splitext(os.path.basename(audio_file))[0]
            title = base
            artist = "unknown"
            album = "unknown"
        
        # Find the corresponding converted files (if they exist)
        # We need to find the index in the converted folder
        audio_filename = os.path.basename(audio_file)
        audio_name, audio_ext = os.path.splitext(audio_filename)
        
        # Look for matching file in converted directory
        converted_wav = None
        converted_img = None
        
        for f in os.listdir(DEST_DIR):
            fpath = os.path.join(DEST_DIR, f)
            if not os.path.isfile(fpath):
                continue
            name, ext = os.path.splitext(f)
            ext = ext.lower()
            
            if ext == ".wav":
                # Try to find the matching WAV file by content, not just name
                # Since we can't easily match, we'll use the track number as index
                if name.isdigit():
                    idx = int(name)
                    # Use the track number as a simple match
                    # This assumes track1 -> 1.wav, track2 -> 2.wav, etc.
                    if idx == track_num:
                        converted_wav = fpath
            elif ext == ".png" and name.isdigit():
                idx = int(name)
                if idx == track_num:
                    converted_img = fpath
        
        # If we found converted files, create track
        if converted_wav and converted_img:
            t = Track(
                title=title,
                artist=artist,
                album=album,
                image_path=converted_img,
                wav_path=converted_wav,
            )
            prepare_track(t, cover_width=320, cover_height=320)
            available_tracks.append((track_num, t))
    
    return available_tracks


# Update the edit_track_metadata_async function to save to input folder
def edit_track_metadata_async(edit_track_queue):
    """
    Non-blocking function to edit track metadata.
    Saves metadata to the input/track*/ folder for persistence.
    """
    print("\n[EDIT] Available tracks to edit:")
    print("-" * 50)
    
    # Scan for available tracks
    available_tracks = scan_available_tracks()
    
    if not available_tracks:
        print("No tracks found. Check if tracks are in the 'input/track*/' folders.")
        return
    
    # Display all tracks with their current metadata
    for i, (idx, track) in enumerate(available_tracks, start=1):
        print(f"  {i}) Track #{idx}: {track.title} - {track.artist} ({track.album})")
    
    print("\nEnter the number of the track to edit (or 0 to cancel):")
    print("(You can continue to use other commands while editing)")
    
    # Use a simple timeout-based input check so we don't block
    import select
    import sys
    
    timeout_count = 0
    max_timeout = 300  # 30 seconds timeout
    
    while timeout_count < max_timeout:
        # Check if input is available without blocking
        if select.select([sys.stdin], [], [], 0.1)[0]:
            choice = sys.stdin.readline().strip()
            
            if choice == "0":
                print("[EDIT] Cancelled.")
                return
            
            if not choice.isdigit():
                print("Please enter a number.")
                # Reset timeout since we got input
                timeout_count = 0
                continue
            
            choice_num = int(choice)
            if not (1 <= choice_num <= len(available_tracks)):
                print(f"Please enter a number between 1 and {len(available_tracks)}.")
                # Reset timeout since we got input
                timeout_count = 0
                continue
            
            selected_idx, selected_track = available_tracks[choice_num - 1]
            
            print(f"\n[EDIT] Editing Track #{selected_idx}:")
            print(f"  Current title:  {selected_track.title}")
            print(f"  Current artist: {selected_track.artist}")
            print(f"  Current album:  {selected_track.album}")
            print("\nEnter new metadata (press Enter to keep current value):")
            
            # Get input for each field
            print("Enter new title (or press Enter to keep current):")
            new_title = None
            for _ in range(150):  # Wait up to 3 seconds for input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    new_title = sys.stdin.readline().strip()
                    break
            
            if new_title is None:
                print("[EDIT] Input timeout. Keeping current title.")
                new_title = selected_track.title
            
            print("Enter new artist (or press Enter to keep current):")
            new_artist = None
            for _ in range(150):  # Wait up to 3 seconds for input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    new_artist = sys.stdin.readline().strip()
                    break
            
            if new_artist is None:
                print("[EDIT] Input timeout. Keeping current artist.")
                new_artist = selected_track.artist
            
            print("Enter new album (or press Enter to keep current):")
            new_album = None
            for _ in range(150):  # Wait up to 3 seconds for input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    new_album = sys.stdin.readline().strip()
                    break
            
            if new_album is None:
                print("[EDIT] Input timeout. Keeping current album.")
                new_album = selected_track.album
            
            # Update the track object
            selected_track.title = new_title if new_title else selected_track.title
            selected_track.artist = new_artist if new_artist else selected_track.artist
            selected_track.album = new_album if new_album else selected_track.album
            
            # Find the input folder for this track
            track_folder = os.path.join(SOURCE_DIR, f"track{selected_idx}")
            
            if os.path.exists(track_folder):
                # Look for existing metadata file
                meta_file = None
                for fname in os.listdir(track_folder):
                    fpath = os.path.join(track_folder, fname)
                    if os.path.isfile(fpath):
                        name, ext = os.path.splitext(fname)
                        ext = ext.lower()
                        if ext in (".txt", ".csv"):
                            meta_file = fpath
                            break
                
                # If no metadata file exists, create one
                if not meta_file:
                    meta_file = os.path.join(track_folder, "meta.txt")
                
                # Save metadata to input folder
                try:
                    with open(meta_file, "w", encoding="utf-8") as f:
                        f.write(f"{selected_track.title}\n{selected_track.artist}\n{selected_track.album}")
                    
                    print(f"\n[EDIT] ✓ Updated metadata for Track #{selected_idx}")
                    print(f"       Saved to: {meta_file}")
                    print(f"       Title:  {selected_track.title}")
                    print(f"       Artist: {selected_track.artist}")
                    print(f"       Album:  {selected_track.album}")
                    
                    # Also update metadata in converted folder for current session
                    # Find the converted metadata file
                    for f in os.listdir(DEST_DIR):
                        fpath = os.path.join(DEST_DIR, f)
                        if not os.path.isfile(fpath):
                            continue
                        name, ext = os.path.splitext(f)
                        ext = ext.lower()
                        if ext == ".txt" and name.isdigit() and int(name) == selected_idx:
                            # Update the converted metadata file too
                            with open(fpath, "w", encoding="utf-8") as f:
                                f.write(f"{selected_track.title}\n{selected_track.artist}\n{selected_track.album}")
                            break
                    
                    # Prepare track with new metadata
                    prepare_track(selected_track, cover_width=320, cover_height=320)
                    
                    # Put the updated track in the queue for the main thread
                    edit_track_queue.put((selected_idx, selected_track))
                    
                except Exception as e:
                    print(f"[EDIT] Error saving metadata: {e}")
            else:
                print(f"[EDIT] Error: Could not find input folder for track {selected_idx}: {track_folder}")
            
            return
        
        else:
            timeout_count += 1
            
            # Periodically show we're still waiting
            if timeout_count % 10 == 0:  # Every 1 second
                print(".", end="", flush=True)
    
    print("\n[EDIT] Input timeout. Cancelling metadata edit.")
    return


# Add this new function for handling track addition in a non-blocking way
def add_track_async(ser, current_queue_tracks, track_to_add_queue, available_tracks_cache):
    """
    Non-blocking function to handle adding tracks.
    Runs in a separate thread to avoid blocking audio processing.
    """
    if not available_tracks_cache:
        available_tracks_cache[:] = scan_available_tracks()
    
    if not available_tracks_cache:
        print("[ADD] No tracks available to add.")
        return
    
    print("\n[ADD] Available tracks to add:")
    print("-" * 50)
    
    # Filter out tracks already in queue
    queue_wav_paths = {t.wav_path for t in current_queue_tracks}
    filtered_tracks = []
    
    for idx, track in available_tracks_cache:
        if track.wav_path not in queue_wav_paths:
            filtered_tracks.append((idx, track))
    
    if not filtered_tracks:
        print("All available tracks are already in the queue!")
        return
    
    for i, (idx, track) in enumerate(filtered_tracks, start=1):
        print(f"  {i}) Track #{idx}: {track.title} - {track.artist}")
    
    print("\nEnter the number of the track to add (or 0 to cancel):")
    print("(You can continue to use other commands while adding tracks)")
    
    # We'll use a simple timeout-based input check so we don't block
    import select
    import sys
    
    timeout_count = 0
    max_timeout = 300  # 30 seconds timeout
    
    while timeout_count < max_timeout:
        # Check if input is available without blocking
        if select.select([sys.stdin], [], [], 0.1)[0]:
            choice = sys.stdin.readline().strip()
            
            if choice == "0":
                print("[ADD] Cancelled.")
                return
            
            if choice.isdigit():
                choice_num = int(choice)
                if 1 <= choice_num <= len(filtered_tracks):
                    selected_idx, selected_track = filtered_tracks[choice_num - 1]
                    print(f"[ADD] Selected Track #{selected_idx}: {selected_track.title}")
                    # Put the track in the queue for the main thread to process
                    track_to_add_queue.put(selected_track)
                    return
                else:
                    print(f"Please enter a number between 1 and {len(filtered_tracks)}.")
            else:
                print("Please enter a number.")
            
            # Reset timeout since we got valid input
            timeout_count = 0
        else:
            timeout_count += 1
            
            # Periodically show we're still waiting
            if timeout_count % 10 == 0:  # Every 1 second
                print(".", end="", flush=True)
    
    print("\n[ADD] Input timeout. Cancelling track addition.")
    return


def prompt_add_track(available_tracks, current_queue_tracks):
    """
    Show available tracks and let user choose which to add.
    Returns selected Track or None if cancelled.
    """
    if not available_tracks:
        print("[ADD] No tracks available to add.")
        return None
    
    print("\n[ADD] Available tracks to add:")
    print("-" * 50)
    
    # Filter out tracks already in queue
    queue_wav_paths = {t.wav_path for t in current_queue_tracks}
    filtered_tracks = []
    
    for idx, track in available_tracks:
        if track.wav_path not in queue_wav_paths:
            filtered_tracks.append((idx, track))
    
    if not filtered_tracks:
        print("All available tracks are already in the queue!")
        return None
    
    for i, (idx, track) in enumerate(filtered_tracks, start=1):
        print(f"  {i}) Track #{idx}: {track.title} - {track.artist}")
    
    print("\nEnter the number of the track to add (or 0 to cancel):")
    
    while True:
        try:
            choice = input("> ").strip()
            if choice == "0":
                return None
            
            if not choice.isdigit():
                print("Please enter a number.")
                continue
                
            choice_num = int(choice)
            if 1 <= choice_num <= len(filtered_tracks):
                selected_idx, selected_track = filtered_tracks[choice_num - 1]
                print(f"[ADD] Selected Track #{selected_idx}: {selected_track.title}")
                return selected_track
            else:
                print(f"Please enter a number between 1 and {len(filtered_tracks)}.")
                
        except (ValueError, KeyboardInterrupt):
            return None


# In the jukebox mode, update the main loop to handle the "add" action
# Look for this section in the jukebox loop and update it:




# ---------- CRC16-CCITT ----------

def crc16_ccitt(data: bytes) -> int:
    remainder = CRC_INIT
    for b in data:
        remainder ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if remainder & 0x8000:
                remainder = ((remainder << 1) ^ CRC_POLY) & 0xFFFF
            else:
                remainder = (remainder << 1) & 0xFFFF
    return remainder





# ---------- Generic menu helpers ----------

def choose_from_list(prompt, options):
    if not options:
        raise RuntimeError(f"No options available for: {prompt}")

    print(prompt)
    for i, opt in enumerate(options, 1):
        print(f"  {i}) {opt}")

    while True:
        choice = input("> ").strip().lower()
        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(options):
                return options[idx - 1]
        print("Please enter a valid number from the list")


def prompt_port():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        sys.exit(1)

    options = [f"{p.device} — {p.description}" for p in ports]
    selected = choose_from_list("Select a COM port:", options)
    return selected.split(" — ", 1)[0]


def prompt_baud():
    baud_options = [
        "115200", "230400", "460800", "921600",
        "1000000", "1500000", "2000000", "2500000", "3000000", "4000000"
    ]
    selected = choose_from_list("Select a baud rate:", baud_options)
    return int(selected)


# ---------- RX worker (reads bytes and sets events) ----------

def rx_worker(ser: serial.Serial):
    global rx_thread_running
    rx_thread_running = True
    log("[RX] Thread started.")
    try:
        while rx_thread_running:
            b = ser.read(1)
            if not b:
                continue  # timeout, keep looping

            if b == b'H':
                header_ack_event.set()
            elif b == b'S':
                ok_ack_event.set()
            elif b == b'A':
                audio_req_event.set()
            elif b == b'E':
                error_event.set()
                if VERBOSE_RX:
                    print("[RX] Received error 'E' from MCU.")
            elif b == b'\xF0':
                btn = ser.read(1)
                if not btn:
                    continue
                if btn == b'P':
                    button_event_queue.put("play_pause")
                    if VERBOSE_RX:
                        print("[RX] Button: PLAY/PAUSE")
                elif btn == b'N':
                    button_event_queue.put("next")
                    if VERBOSE_RX:
                        print("[RX] Button: NEXT")
                elif btn == b'B':
                    button_event_queue.put("prev")
                    if VERBOSE_RX:
                        print("[RX] Button: PREV")
                else:
                    if VERBOSE_RX:
                        print(f"[RX] Unknown button code: {btn!r}")
    finally:
        log("[RX] Thread exiting.")



# ---------- Event-based wait helpers ----------

def wait_header_ack(label: str = "", timeout: float = 1.0):
    if not header_ack_event.wait(timeout=timeout):
        raise TimeoutError(f"Timeout waiting for header ACK 'H' ({label})")
    header_ack_event.clear()


def wait_ok_or_error(label: str = "", timeout: float = 10.0) -> bool:
    """
    Wait for either 'S' (OK) or 'E' (error/NACK).
    Returns True on OK, False on error.
    Raises TimeoutError if nothing arrives in time.
    """
    deadline = time.time() + timeout
    while True:
        if ok_ack_event.is_set():
            ok_ack_event.clear()
            return True
        if error_event.is_set():
            error_event.clear()
            return False
        if time.time() >= deadline:
            raise TimeoutError(f"Timeout waiting for OK/ERR ('S' or 'E') ({label})")
        time.sleep(0.001)


def wait_audio_req(label: str = "", timeout: float = None):
    """
    Wait for 'A' (next audio chunk). If timeout is None, wait forever.
    """
    if not audio_req_event.wait(timeout=timeout):
        raise TimeoutError(f"Timeout waiting for audio request 'A' ({label})")
    audio_req_event.clear()


def clear_all_events():
    header_ack_event.clear()
    ok_ack_event.clear()
    audio_req_event.clear()
    error_event.clear()
    # do not touch button_event_queue here; jukebox loop will drain it


# ---------- Image-specific helpers ----------

def prompt_image_file():
    exts = (".png", ".jpg", ".jpeg", ".bmp")
    imgs = [f for f in os.listdir(".") if f.lower().endswith(exts)]
    imgs.sort()
    if imgs:
        return choose_from_list("Select an image file from the current folder:", imgs)
    print("No image files (.png/.jpg/.jpeg/.bmp) found in current directory.")
    sys.exit(1)


def prompt_dimensions(default_w=480, default_h=320):
    print(f"Target dimensions (enter to accept default {default_w}x{default_h})")
    w_str = input("Width  (pixels): ").strip()
    h_str = input("Height (pixels): ").strip()

    w = default_w if not w_str else int(w_str)
    h = default_h if not h_str else int(h_str)
    return w, h


def image_to_rgb565_be_bytes(image_path: str, W: int, H: int) -> bytes:
    """
    Open image, convert to RGB, resize to (W, H),
    and return bytes in row-major order, RGB565 big-endian per pixel.
    """
    im = Image.open(image_path).convert('RGB').resize((W, H), Image.LANCZOS)

    out = bytearray()
    for y in range(H):
        for x in range(W):
            r, g, b = im.getpixel((x, y))
            rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            out.extend(struct.pack('>H', rgb565))  # big-endian
    return bytes(out)  # length = W * H * 2


def build_image_payload(x: int, y: int, w: int, h: int, pixel_bytes: bytes) -> bytes:
    """
    [x,y,w,h] (u16 BE) + w*h*2 bytes of RGB565
    """
    header = struct.pack('>4H', x, y, w, h)  # big-endian 16-bit fields
    return header + pixel_bytes


def compute_max_rows_per_packet(W: int) -> int:
    """
    Given width W, find maximum tile height so that:
      8 (x,y,w,h) + 2*W*h <= MAX_DATA_SIZE
    """
    max_rows = (MAX_DATA_SIZE - 8) // (2 * W)
    if max_rows <= 0:
        raise ValueError("Width too large to fit any rows in a packet.")
    return int(max_rows)


# ---------- Audio-specific helpers ----------

def prompt_wav_file():
    wavs = [f for f in os.listdir(".") if f.lower().endswith(".wav")]
    wavs.sort()
    if wavs:
        return choose_from_list("Select a WAV file from the current folder:", wavs)
    print("No .wav files found in the current directory.")
    sys.exit(1)


def load_wav_mono_16bit(path: str):
    """
    Load a WAV and return (samples_int16_little_endian, sample_rate).
    If stereo, average to mono.
    """
    with wave.open(path, 'rb') as wf:
        n_channels = wf.getnchannels()
        sampwidth  = wf.getsampwidth()
        n_frames   = wf.getnframes()
        framerate  = wf.getframerate()

        raw = wf.readframes(n_frames)

    if sampwidth != 2:
        raise ValueError("Expected 16-bit PCM WAV")

    # Interpret as int16 little-endian
    data = np.frombuffer(raw, dtype='<i2')  # < = little-endian

    if n_channels == 2:
        data = data.reshape(-1, 2).mean(axis=1).astype('<i2')

    return data, framerate


# ---------- Packet builder ----------

def build_packet(data_type: int,
                 flags: int,
                 seq: int,
                 payload: bytes) -> bytes:
    """
    Mirrors your pkt_build on the PC side.
    """
    if len(payload) > MAX_DATA_SIZE:
        raise ValueError(f"Payload too large for single packet: {len(payload)} bytes")

    buf = bytearray()
    buf.append(PACKET_SOF0)
    buf.append(PACKET_SOF1)

    # Header fields
    version   = PACKET_VERSION
    seq_high  = (seq >> 8) & 0xFF
    seq_low   = seq & 0xFF
    len_field = len(payload)
    len_high  = (len_field >> 8) & 0xFF
    len_low   = len_field & 0xFF

    buf.append(version)
    buf.append(data_type & 0xFF)
    buf.append(flags & 0xFF)
    buf.append(seq_high)
    buf.append(seq_low)
    buf.append(len_high)
    buf.append(len_low)

    # Payload
    buf.extend(payload)

    # Compute CRC over [version..end of payload]
    crc_input = bytes(buf[2:])  # same as &buf[2] in C
    crc = crc16_ccitt(crc_input)
    crc_high = (crc >> 8) & 0xFF
    crc_low  = crc & 0xFF

    buf.append(crc_high)
    buf.append(crc_low)

    return bytes(buf)


# ---------- Core send function using events ----------

def send_packet_with_handshake(ser: serial.Serial,
                               data_type: int,
                               flags: int,
                               seq: int,
                               payload: bytes,
                               label: str = "",
                               timeout: float = 2.0) -> bool:
    pkt = build_packet(data_type, flags, seq, payload)
    header = pkt[:9]
    body   = pkt[9:]

    header_ack_event.clear()
    with TX_LOCK:
        ser.write(header)
        ser.flush()
    if VERBOSE_TX:
        print(f"[TX] Header sent ({label}, len={len(header)})")

    try:
        wait_header_ack(label=label)
    except TimeoutError as e:
        print(f"[TX] ERROR: {e}")
        return False

    with TX_LOCK:
        ser.write(body)
        ser.flush()
    if VERBOSE_TX:
        print(f"[TX] Body sent   ({label}, len={len(body)})")

    try:
        ok = wait_ok_or_error(label=label, timeout=timeout)
    except TimeoutError as e:
        print(f"[TX] ERROR: {e}")
        return False

    if not ok:
        print(f"[TX] MCU reported error for {label}")
        return False

    return True



# ---------- Packet preparation (no I/O) ----------

def make_image_packets(image_path: str, W: int, H: int) -> List[PacketEntry]:
    """
    Precompute all image packets (seq, flags, payload, debug_label).
    """
    log(f"Converting image '{image_path}' to {W}x{H} RGB565...")
    full_bytes = image_to_rgb565_be_bytes(image_path, W, H)
    log(f"Image converted: {len(full_bytes)} bytes total.")

    max_rows = compute_max_rows_per_packet(W)
    log(f"Max rows per packet at width {W}: {max_rows}")

    packets: List[PacketEntry] = []
    seq = 0
    y0 = 0
    while y0 < H:
        h_tile = min(max_rows, H - y0)
        tile_px_bytes = 2 * W * h_tile
        start = (y0 * W * 2)
        chunk = full_bytes[start:start + tile_px_bytes]

        x = 0
        w = W
        payload = build_image_payload(x, y0, w, h_tile, chunk)
        flags = PKT_FLAG_LAST if (y0 + h_tile >= H) else 0

        label = f"IMG seq={seq}, y={y0}, h={h_tile}"
        packets.append((seq, flags, payload, label))

        seq = (seq + 1) & 0xFFFF
        y0 += h_tile

    log(f"Prepared {len(packets)} image packets.")
    return packets


def make_audio_packets(wav_path: str) -> List[PacketEntry]:
    """
    Precompute all audio packets (seq, flags, payload, debug_label).
    Payload size is limited to AUDIO_MAX_PAYLOAD to match STM32.
    """
    log(f"Loading WAV file '{wav_path}'...")
    samples, fs = load_wav_mono_16bit(wav_path)
    log(f"WAV loaded: {len(samples)} samples @ {fs} Hz.")

    raw = samples.tobytes()
    total_bytes = len(raw)
    log(f"Total audio bytes: {total_bytes}")

    packets: List[PacketEntry] = []
    seq = 0
    offset = 0
    pkt_index = 0

    while offset < total_bytes:
        remaining = total_bytes - offset
        this_len  = min(remaining, AUDIO_MAX_PAYLOAD)
        payload   = raw[offset:offset + this_len]
        offset   += this_len

        flags = 0
        if offset >= total_bytes:
            flags |= PKT_FLAG_LAST

        label = f"AUD seq={seq}, idx={pkt_index}, bytes={len(payload)}"
        packets.append((seq, flags, payload, label))

        pkt_index += 1
        seq = (seq + 1) & 0xFFFF

    log(f"Prepared {len(packets)} audio packets.")
    return packets


# ---------- Text helpers ----------

def make_text_payload(field_id: int, text: str) -> bytes:
    """
    Payload layout:
      [0]   field_id
      [1..] UTF-8 bytes of text (no NUL)
    """
    encoded = text.encode("utf-8")
    if 1 + len(encoded) > MAX_DATA_SIZE:
        raise ValueError(f"Text too long for one packet ({len(encoded)} bytes)")

    return bytes([field_id & 0xFF]) + encoded


def send_text_field(ser: serial.Serial, field_id: int, text: str,
                    seq: int = 0, label_prefix: str = "TEXT"):
    payload = make_text_payload(field_id, text)
    flags = PKT_FLAG_LAST  # single-packet text, so mark as LAST

    label = f"{label_prefix} field={field_id}"
    ok = send_packet_with_handshake(
        ser,
        DATA_TYPE_TEXT,   # -> PKT_DATA_TEXT on MCU
        flags,
        seq,
        payload,
        label=label,
        timeout=2.0,
    )
    if not ok:
        print(f"[TEXT] Failed to send {label}")
    else:
        print(f"[TEXT] Sent {label}: {text!r}")
    return ok


def prompt_and_send_metadata(ser: serial.Serial, seq_start: int = 0) -> int:
    """
    Prompt user for title/artist/album and send as text packets.
    Returns next available seq value (seq_start + number of fields sent).
    """
    print("\nEnter track metadata for this image (leave blank to skip a field):")
    title  = input("  Title : ").strip()
    artist = input("  Artist: ").strip()
    album  = input("  Album : ").strip()

    seq = seq_start

    if title:
        send_text_field(ser, TEXT_FIELD_TITLE, title, seq=seq)
        seq = (seq + 1) & 0xFFFF

    if artist:
        send_text_field(ser, TEXT_FIELD_ARTIST, artist, seq=seq)
        seq = (seq + 1) & 0xFFFF

    if album:
        send_text_field(ser, TEXT_FIELD_ALBUM, album, seq=seq)
        seq = (seq + 1) & 0xFFFF

    if not (title or artist or album):
        print("[TEXT] No metadata entered; skipping text packets.")
    else:
        print("[TEXT] Metadata send complete.\n")

    return seq


# ---------- High-level streaming modes (legacy/test) ----------

def send_image_only(ser: serial.Serial, image_path: str, W: int, H: int):
    clear_all_events()
    img_packets = make_image_packets(image_path, W, H)

    print("\n[MODE] Image-only transfer")
    for seq, flags, payload, label in img_packets:
        if not send_packet_with_handshake(ser, PKT_DATA_IMAGE, flags, seq, payload, label):
            print("[MODE] Aborting image transfer due to error.")
            return
    print("[MODE] Image-only transfer complete.")


def send_album_cover(ser: serial.Serial, image_path: str):
    """
    Album cover mode:
      - Always sends a 320x320 image at x=0,y=0
      - Intended for the top of a 320x480 vertical display
    """
    W, H = 320, 320
    clear_all_events()
    img_packets = make_image_packets(image_path, W, H)

    print("\n[MODE] Album cover (320x320 at top)")
    for seq, flags, payload, label in img_packets:
        if not send_packet_with_handshake(ser, PKT_DATA_IMAGE, flags, seq, payload, label):
            print("[MODE] Aborting album cover transfer due to error.")
            return
    print("[MODE] Album cover transfer complete.")


def send_audio_stop(ser: serial.Serial):
    payload = bytes([CTRL_CMD_AUDIO_STOP])
    send_packet_with_handshake(ser, PKT_DATA_CTRL, 0, 0, payload, "CTRL_AUDIO_STOP")


def send_audio_only(ser: serial.Serial, wav_path: str):
    """
    Audio-only mode with a 'stop button' via ENTER in terminal.
    This is left as a test/debug mode.
    """
    clear_all_events()
    aud_packets = make_audio_packets(wav_path)
    if not aud_packets:
        print("No audio packets to send.")
        return

    print("\n[MODE] Audio-only transfer")
    print("Press ENTER at any time to stop playback early...")

    stop_event = threading.Event()

    def stop_listener():
        try:
            input()
        except EOFError:
            return
        stop_event.set()
        print("\n[AUDIO] Stop requested – will finish current packet,")
        print("[AUDIO] send one LAST packet, then CTRL_AUDIO_STOP.\n")

    listener_thread = threading.Thread(target=stop_listener, daemon=True)
    listener_thread.start()

    stopping_mode = False

    try:
        for idx, (seq, flags, payload, label) in enumerate(aud_packets):
            if idx > 0:
                wait_audio_req(label=f"audio idx={idx}")

            if stop_event.is_set() and not stopping_mode:
                stopping_mode = True
                flags |= PKT_FLAG_LAST
                label = f"{label} [FORCED_LAST]"

            ok = send_packet_with_handshake(
                ser,
                PKT_DATA_AUDIO,
                flags,
                seq,
                payload,
                label,
                timeout=10.0,
            )
            if not ok:
                print("[AUDIO] Aborting due to error on packet", idx)
                return

            if flags & PKT_FLAG_LAST:
                print("[AUDIO] Sent LAST packet, ending stream of audio packets.")
                break

    except KeyboardInterrupt:
        print("\n[Audio] KeyboardInterrupt detected. Aborting audio transfer WITHOUT sending CTRL_AUDIO_STOP.")
        print("[Audio] MCU UART state may be mid-packet; you may need to reset the board.")
        return

    print("[AUDIO] Sending AUDIO_STOP control packet...")
    send_audio_stop(ser)
    print("[AUDIO] Audio-only transfer complete with clean STOP.")


def send_image_and_audio_interleaved(ser: serial.Serial,
                                     image_path: str,
                                     W: int,
                                     H: int,
                                     wav_path: str):
    """
    Event-loop style:
      - Precompute all image + audio packets.
      - Send first audio packet immediately.
      - Then loop:
          * if 'A' pending and audio packets left:
              send next audio packet
          * else if image packets left:
              send one image packet
          * else:
              done
    """
    clear_all_events()
    img_packets = make_image_packets(image_path, W, H)
    aud_packets = make_audio_packets(wav_path)

    log("\n[MODE] Interleaved Image + Audio")

    if not aud_packets:
        print("No audio packets, falling back to image-only.")
        for seq, flags, payload, label in img_packets:
            if not send_packet_with_handshake(ser, PKT_DATA_IMAGE, flags, seq, payload, label):
                print("[MODE] Aborting interleaved transfer due to error.")
                return
        return

    # First audio packet: send immediately (no 'A' gating)
    aud_idx = 0
    seq, flags, payload, label = aud_packets[aud_idx]
    log("[INTERLEAVE] Sending first audio packet immediately.")
    if not send_packet_with_handshake(ser, PKT_DATA_AUDIO, flags, seq, payload, label):
        log("[MODE] Aborting interleaved transfer due to error on first audio packet.")
        return
    aud_idx += 1

    img_idx = 0
    total_img = len(img_packets)
    total_aud = len(aud_packets)

    while img_idx < total_img or aud_idx < total_aud:
        did_something = False

        if aud_idx < total_aud and audio_req_event.is_set():
            audio_req_event.clear()
            seq, flags, payload, label = aud_packets[aud_idx]
            log(f"[INTERLEAVE] MCU requested audio -> {label}")
            if not send_packet_with_handshake(ser, PKT_DATA_AUDIO, flags, seq, payload, label):
                log("[MODE] Aborting interleaved transfer due to audio error.")
                return
            aud_idx += 1
            did_something = True
            continue

        if img_idx < total_img:
            seq, flags, payload, label = img_packets[img_idx]
            log(f"[INTERLEAVE] Sending image tile -> {label}")
            if not send_packet_with_handshake(ser, PKT_DATA_IMAGE, flags, seq, payload, label):
                log("[MODE] Aborting interleaved transfer due to image error.")
                return
            img_idx += 1
            did_something = True
            continue

        if aud_idx < total_aud and not did_something:
            time.sleep(0.001)

    log("[MODE] Interleaved Image + Audio complete.")


# ---------- Track preparation & UI helpers ----------

def prepare_track(track: Track,
                  cover_width: int = 320,
                  cover_height: int = 320) -> None:
    """
    Fill track.image_packets and track.audio_packets using existing helpers.
    """
    if track.image_path:
        track.image_packets = make_image_packets(track.image_path, cover_width, cover_height)
    else:
        track.image_packets = []

    track.audio_packets = make_audio_packets(track.wav_path)
    track.audio_index = 0
    track.finished = False


def display_track_ui(ser: serial.Serial, track: Track) -> None:
    """
    Draw track's cover art and metadata on the TFT using packets.
    """
    print(f"[UI] Displaying track UI: '{track.title}' by '{track.artist}'")

    # Send image packets
    for seq, flags, payload, label in track.image_packets:
        if not send_packet_with_handshake(ser, PKT_DATA_IMAGE, flags, seq, payload, label):
            print("[UI] Aborting image transfer for this track due to error.")
            break

    # Send text fields (we don't care much about seq here; reuse 0,1,2)
    seq = 0
    if track.title:
        send_text_field(ser, TEXT_FIELD_TITLE, track.title, seq=seq, label_prefix="TITLE")
        seq = (seq + 1) & 0xFFFF
    if track.artist:
        send_text_field(ser, TEXT_FIELD_ARTIST, track.artist, seq=seq, label_prefix="ARTIST")
        seq = (seq + 1) & 0xFFFF
    if track.album:
        send_text_field(ser, TEXT_FIELD_ALBUM, track.album, seq=seq, label_prefix="ALBUM")

def clear_converted_dir():
    """
    Remove all files/subdirectories from DEST_DIR but keep the folder itself.
    Also reset watcher bookkeeping so files in input/ can be reprocessed.
    """
    if not os.path.isdir(DEST_DIR):
        return

    for name in os.listdir(DEST_DIR):
        path = os.path.join(DEST_DIR, name)
        try:
            if os.path.isfile(path) or os.path.islink(path):
                os.remove(path)
            elif os.path.isdir(path):
                shutil.rmtree(path)
        except Exception as e:
            log(f"[CLEAN] Failed to delete {path}: {e}")

    # Reset watcher state so existing input/ files can be converted again
    processed_files.clear()
    waiting_audio.clear()
    waiting_images.clear()

    log(f"[CLEAN] Cleared '{DEST_DIR}' and reset watcher state.")



def build_tracks_from_converted_dir() -> List[Track]:
    """
    Scan DEST_DIR for numbered pairs N.wav + N.png and turn them into Track objects.
    Title defaults to 'Track N', artist/album 'unknown'.
    """
    if not os.path.isdir(DEST_DIR):
        return []

    files = os.listdir(DEST_DIR)
    stems = {}

    for f in files:
        path = os.path.join(DEST_DIR, f)
        if not os.path.isfile(path):
            continue
        name, ext = os.path.splitext(f)
        ext = ext.lower()
        if not name.isdigit():
            continue
        idx = int(name)
        if idx not in stems:
            stems[idx] = {}
        if ext == ".wav":
            stems[idx]["wav"] = path
        elif ext == ".png":
            stems[idx]["img"] = path

    tracks: List[Track] = []
    for idx in sorted(stems.keys()):
        entry = stems[idx]
        if "wav" in entry and "img" in entry:
            wav_path = entry["wav"]
            img_path = entry["img"]
            title = f"Track {idx}"
            artist = "unknown"
            album = "unknown"
            t = Track(
                title=title,
                artist=artist,
                album=album,
                image_path=img_path,
                wav_path=wav_path,
            )
            print(f"[JUKEBOX] Found auto track #{idx}: wav={wav_path}, img={img_path}")
            prepare_track(t, cover_width=320, cover_height=320)
            tracks.append(t)

    return tracks


# ---------- Jukebox mode (playlist + hardware buttons) ----------

def run_jukebox(ser: serial.Serial):
    """
    Jukebox mode:
      - Discover auto tracks from:
          * input/<track folders> (audio + image + optional metadata)
          * converted/N.wav + converted/N.png pairs
      - Let the user choose which auto tracks to add to the playlist.
      - Or fall back to a manual playlist builder.
      - Use MCU buttons (next/prev/play-pause) to control playback.
      - Respond to 'A' requests to stream audio chunks.
    """
    print("\n=== Jukebox mode ===")
    print("This mode will play a playlist and respond to hardware buttons.")
    print("Album art will be sent as 320x320 cover at top of the display.")
    print("Press Ctrl+C to exit jukebox mode and return to the main menu.\n")

    # ---- Reset converted/ for a fresh jukebox session ----
    clear_converted_dir()

    # ----- Build auto tracks -----
    folder_tracks = build_converted_from_input_folders()   # uses input/<track> folders
    dest_tracks   = build_tracks_from_converted_dir()      # uses converted/N.wav + N.png

    # Avoid double-counting the same converted WAV paths
    seen_wavs = {t.wav_path for t in folder_tracks}
    extra_dest_tracks: List[Track] = []
    for t in dest_tracks:
        if t.wav_path not in seen_wavs:
            extra_dest_tracks.append(t)

    auto_tracks: List[Track] = folder_tracks + extra_dest_tracks
    queue_tracks: List[Track] = []

    # ----- Let user choose which auto tracks to add -----
    if auto_tracks:
        print("[JUKEBOX] Discovered the following auto tracks:")
        for i, t in enumerate(auto_tracks, start=1):
            print(f"  {i}) {t.title} – {t.artist}  ({os.path.basename(t.wav_path)})")

        print("\nEnter the numbers of the tracks you want to add to the playlist,")
        print("comma-separated, in the order you want them to play (e.g. 3,1,2).")
        print("Press ENTER with no input to skip auto tracks and build a manual playlist.")

        sel = input("> ").strip()
        if sel:
            indices = []
            for chunk in sel.replace(" ", "").split(","):
                if not chunk:
                    continue
                if not chunk.isdigit():
                    print(f"  Ignoring invalid entry '{chunk}' (not a number).")
                    continue
                idx = int(chunk)
                if 1 <= idx <= len(auto_tracks):
                    indices.append(idx - 1)
                else:
                    print(f"  Ignoring out-of-range index {idx}.")

            # Remove duplicates while preserving order
            seen = set()
            unique_indices = []
            for i in indices:
                if i not in seen:
                    seen.add(i)
                    unique_indices.append(i)

            if unique_indices:
                for i in unique_indices:
                    queue_tracks.append(auto_tracks[i])
    else:
        print("[JUKEBOX] No auto tracks found yet (check 'input/' and 'converted/').")

    # ----- Manual playlist configuration if needed -----
    if not queue_tracks:
        print("\n[JUKEBOX] No auto tracks selected; falling back to manual playlist builder.")
        while True:
            try:
                num_str = input("How many tracks do you want to add to the playlist? ")
                num = int(num_str.strip())
                if num <= 0:
                    print("No tracks requested; leaving jukebox mode.")
                    return
                break
            except ValueError:
                print("Please enter a valid integer.")

        for i in range(num):
            print(f"\n--- Configure track {i + 1} of {num} ---")
            image_path = prompt_image_file()
            wav_path   = prompt_wav_file()

            default_title = os.path.splitext(os.path.basename(wav_path))[0]
            print("Enter metadata (leave blank to accept defaults):")
            title  = input(f"  Title  [{default_title}]: ").strip() or default_title
            artist = input("  Artist [unknown]: ").strip() or "unknown"
            album  = input("  Album  [unknown]: ").strip() or "unknown"

            track = Track(
                title=title,
                artist=artist,
                album=album,
                image_path=image_path,
                wav_path=wav_path,
            )

            prepare_track(track, cover_width=320, cover_height=320)
            queue_tracks.append(track)

    if not queue_tracks:
        print("[JUKEBOX] No tracks prepared; returning to main menu.")
        return

    print(f"\n[JUKEBOX] Prepared {len(queue_tracks)} tracks. Starting playback...\n")

    # ----- Playlist + player state -----
    history: List[Track] = []
    current_track: Optional[Track] = None
    player_state: str = "IDLE"  # "IDLE", "PLAYING", "PAUSED"

    clear_all_events()

    # ----- NEW: Track addition state -----
    adding_track_thread = None
    track_to_add_queue: Queue[Track] = Queue()
    available_tracks_cache = []

    # ----- NEW: Track editing state -----
    editing_track_thread = None
    edited_track_queue: Queue[Tuple[int, Track]] = Queue()

    def start_track(track: Track):
        nonlocal current_track, player_state
        print(f"[JUKEBOX] Starting track: {track.title} – {track.artist}")
        current_track = track
        current_track.audio_index = 0
        current_track.finished = False
        clear_all_events()

        display_track_ui(ser, current_track)

        if current_track.audio_packets:
            seq, flags, payload, label = current_track.audio_packets[current_track.audio_index]
            print(f"[JUKEBOX] Sending first audio packet -> {label}")
            ok = send_packet_with_handshake(
                ser, PKT_DATA_AUDIO, flags, seq, payload, label, timeout=10.0
            )
            if not ok:
                print("[JUKEBOX] Error sending first audio packet; track may not play correctly.")
            if (flags & PKT_FLAG_LAST) != 0:
                current_track.finished = True
            current_track.audio_index += 1
            player_state = "PLAYING"
        else:
            print("[JUKEBOX] Track has no audio packets; marking finished.")
            current_track.finished = True
            player_state = "IDLE"

        print_jukebox_ui(current_track, queue_tracks, history, player_state)


    def go_to_next_track():
        nonlocal current_track, player_state, queue_tracks, history

        if current_track is not None:
            print(f"[JUKEBOX] Skipping current track: {current_track.title}")
            history.append(current_track)
            # Clean stop for the STM32
            send_audio_stop(ser)

        if queue_tracks:
            next_track = queue_tracks.pop(0)
            start_track(next_track)
        else:
            print("[JUKEBOX] No more tracks in queue.")
            current_track = None
            player_state = "IDLE"

        print_jukebox_ui(current_track, queue_tracks, history, player_state)

    def go_to_prev_track():
        nonlocal current_track, player_state, queue_tracks, history

        if not history:
            print("[JUKEBOX] No previous track in history.")
            return

        if current_track is not None:
            print(f"[JUKEBOX] Moving current track back to queue: {current_track.title}")
            queue_tracks.insert(0, current_track)
            send_audio_stop(ser)

        prev = history.pop()
        start_track(prev)

        print_jukebox_ui(current_track, queue_tracks, history, player_state)

    def handle_play_pause():
        nonlocal current_track, player_state

        if current_track is None:
            if queue_tracks:
                print("[JUKEBOX] No current track; starting next from queue.")
                go_to_next_track()
            else:
                print("[JUKEBOX] No tracks available to play/pause.")
            return

        if player_state == "PLAYING":
            print("[JUKEBOX] Pausing playback.")
            player_state = "PAUSED"
            # Do NOT send AUDIO_STOP; just stop feeding new audio.
        elif player_state == "PAUSED":
            print("[JUKEBOX] Resuming playback.")
            player_state = "PLAYING"
        else:
            print(f"[JUKEBOX] Play/pause pressed in state={player_state}; ignoring.")
        
        print_jukebox_ui(current_track, queue_tracks, history, player_state)

    def send_next_audio_packet_for_current_track():
        nonlocal current_track
        if current_track is None:
            return
        if current_track.audio_index >= len(current_track.audio_packets):
            # No more packets
            return

        seq, flags, payload, label = current_track.audio_packets[current_track.audio_index]
        log(f"[JUKEBOX] Sending audio packet idx={current_track.audio_index} -> {label}")
        ok = send_packet_with_handshake(
            ser, PKT_DATA_AUDIO, flags, seq, payload, label, timeout=10.0
        )
        if not ok:
            print("[JUKEBOX] Error sending audio packet; stream may glitch.")
        if (flags & PKT_FLAG_LAST) != 0:
            log("[JUKEBOX] Last audio packet sent for this track.")
            current_track.finished = True
        current_track.audio_index += 1

    # Start UI input thread (reads from stdin and pushes to command_queue)
    def ui_input_worker():
        while True:
            try:
                cmd = input("> ").strip("\n")
            except EOFError:
                break
            command_queue.put(cmd)

    threading.Thread(target=ui_input_worker, daemon=True).start()

    # Initial UI draw
    print_jukebox_ui(current_track, queue_tracks, history, player_state)



    # ----- Main jukebox loop -----
    try:
        while True:
            # A. Start first track if idle and we have tracks
            if player_state == "IDLE" and current_track is None and queue_tracks:
                go_to_next_track()

            # B. Handle button events from MCU
            while True:
                try:
                    evt = button_event_queue.get_nowait()
                except Empty:
                    break

                if evt == "next":
                    go_to_next_track()
                elif evt == "prev":
                    go_to_prev_track()
                elif evt == "play_pause":
                    handle_play_pause()

            # B2. Handle text UI commands from keyboard
            while True:
                try:
                    cmd = command_queue.get_nowait()
                except Empty:
                    break

                action, args = parse_ui_command(
                    cmd, current_track, queue_tracks, history, player_state
                )
                if action == "next":
                    go_to_next_track()
                elif action == "prev":
                    go_to_prev_track()
                elif action == "toggle":
                    handle_play_pause()
                elif action == "print":
                    print_jukebox_ui(current_track, queue_tracks, history, player_state)
                elif action == "add":
                    # Check if we're already adding a track
                    if adding_track_thread and adding_track_thread.is_alive():
                        print("[ADD] Already adding a track. Please wait or cancel.")
                    else:
                        # Start non-blocking track addition
                        adding_track_thread = threading.Thread(
                            target=add_track_async,
                            args=(ser, queue_tracks, track_to_add_queue, available_tracks_cache),
                            daemon=True
                        )
                        adding_track_thread.start()
                        print("[ADD] Started track addition in background. Continue using other commands.")
                elif action == "delete":
                    idx = args
                    if 1 <= idx <= len(queue_tracks):
                        removed = queue_tracks.pop(idx - 1)
                        print(f"[UI] Removed from queue: {removed.title} – {removed.artist}")
                        print_jukebox_ui(current_track, queue_tracks, history, player_state)
                    else:
                        print(f"[UI] Invalid delete index {idx}.")
                elif action == "move":
                    src, dst = args
                    if 1 <= src <= len(queue_tracks) and 1 <= dst <= len(queue_tracks):
                        item = queue_tracks.pop(src - 1)
                        queue_tracks.insert(dst - 1, item)
                        print(f"[UI] Moved '{item.title}' from {src} to {dst}.")
                        print_jukebox_ui(current_track, queue_tracks, history, player_state)
                    else:
                        print(f"[UI] Invalid move indices {src} -> {dst}.")
                elif action == "edit":
                    # Check if we're already editing a track
                    if editing_track_thread and editing_track_thread.is_alive():
                        print("[EDIT] Already editing a track. Please wait or cancel.")
                    else:
                        # Start non-blocking track editing
                        editing_track_thread = threading.Thread(
                            target=edit_track_metadata_async,
                            args=(edited_track_queue,),
                            daemon=True
                        )
                        editing_track_thread.start()
                        print("[EDIT] Started metadata editor in background. Continue using other commands.")
                elif action == "quit":
                    print("[JUKEBOX] Quit command received.")
                    send_audio_stop(ser)
                    return

            # C. Check for newly added tracks from the async thread
            try:
                while True:
                    new_track = track_to_add_queue.get_nowait()
                    queue_tracks.append(new_track)
                    print(f"\n[ADD] Added '{new_track.title}' to the end of the queue.")
                    print_jukebox_ui(current_track, queue_tracks, history, player_state)
            except Empty:
                pass

            # D. Handle 'A' (next audio chunk) only when playing
            if player_state == "PLAYING" and current_track is not None:
                if audio_req_event.is_set():
                    audio_req_event.clear()
                    send_next_audio_packet_for_current_track()

            # E. End-of-track auto-advance
            if player_state == "PLAYING" and current_track is not None and current_track.finished:
                if queue_tracks:
                    print("[JUKEBOX] Track finished; advancing to next.")
                    go_to_next_track()
                else:
                    print("[JUKEBOX] Track finished; playlist done.")
                    current_track = None
                    player_state = "IDLE"
                    print_jukebox_ui(current_track, queue_tracks, history, player_state)

            # Check for edited tracks from the async thread
            try:
                while True:
                    track_idx, updated_track = edited_track_queue.get_nowait()
                    print(f"\n[EDIT] Updated metadata for Track #{track_idx}: {updated_track.title}")
                    
                    # Update any instances of this track in the queue
                    for i, track in enumerate(queue_tracks):
                        if track.wav_path == updated_track.wav_path:
                            queue_tracks[i] = updated_track
                    
                    # Update history if this track is in history
                    for i, track in enumerate(history):
                        if track.wav_path == updated_track.wav_path:
                            history[i] = updated_track
                    
                    # Update current track if it's the same track
                    if current_track and current_track.wav_path == updated_track.wav_path:
                        current_track = updated_track
                        print(f"[EDIT] Current track updated with new metadata.")
                    
                    print_jukebox_ui(current_track, queue_tracks, history, player_state)
            except Empty:
                pass


            time.sleep(0.001)


    except KeyboardInterrupt:
        print("\n[JUKEBOX] Exiting jukebox mode and returning to the main menu.")
        return

# ---------- Main ----------

def main():
    global rx_thread_running

    # Start the auto-conversion watcher in the background
    watcher_thread = threading.Thread(
        target=conversion_watcher_loop,
        daemon=True,
    )
    watcher_thread.start()

    port = prompt_port()
    baud = prompt_baud()

    print("\n=== Configuration ===")
    print(f"Port : {port}")
    print(f"Baud : {baud}")
    print("=====================\n")

    with serial.Serial(port, baud, timeout=0.1) as ser:
        ser.reset_input_buffer()

        # Start RX thread once; it services all modes
        rx_thread = threading.Thread(target=rx_worker, args=(ser,), daemon=True)
        rx_thread.start()

        try:
            while True:
                mode = choose_from_list(
                    "\nWhat do you want to do now?",
                    [
                        "Image over packets",
                        "Album cover (320x320, vertical top)",
                        "Audio WAV over packets",
                        "Image + Audio (interleaved)",
                        "Jukebox mode (playlist + hardware buttons)",
                        "Exit",
                    ],
                )

                if mode.startswith("Image over"):
                    image_path = prompt_image_file()
                    W, H = prompt_dimensions(default_w=480, default_h=320)
                    print(f"\nImage file : {image_path}")
                    print(f"Target size: {W} x {H}\n")
                    send_image_only(ser, image_path, W, H)
                    prompt_and_send_metadata(ser, seq_start=0)

                elif mode.startswith("Album cover"):
                    image_path = prompt_image_file()
                    print(f"\nAlbum cover file : {image_path}")
                    print("Target size: 320 x 320 (fixed for top of vertical display)\n")
                    send_album_cover(ser, image_path)
                    prompt_and_send_metadata(ser, seq_start=0)

                elif mode.startswith("Audio WAV"):
                    wav_path = prompt_wav_file()
                    print(f"\nWAV file: {wav_path}\n")
                    send_audio_only(ser, wav_path)

                elif mode.startswith("Image + Audio"):
                    image_path = prompt_image_file()
                    W, H = prompt_dimensions(default_w=320, default_h=320)
                    print(f"\nImage file : {image_path}")
                    print(f"Target size: {W} x {H}\n")

                    wav_path = prompt_wav_file()
                    print(f"\nWAV file: {wav_path}\n")

                    send_image_and_audio_interleaved(ser, image_path, W, H, wav_path)

                elif mode.startswith("Jukebox mode"):
                    run_jukebox(ser)

                else:
                    print("Exiting.")
                    break

        finally:
            rx_thread_running = False
            time.sleep(0.1)


if __name__ == "__main__":
    print(">>> entering main()")
    try:
        main()
    except KeyboardInterrupt:
        print("\nCanceled by user.")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    print(">>> main() returned")
