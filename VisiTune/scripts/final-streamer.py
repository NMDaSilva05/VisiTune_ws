import os
import sys
import struct
import wave
import time
import threading
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from queue import Queue, Empty

import serial
from serial.tools import list_ports
from PIL import Image
import numpy as np

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
    print("[RX] Thread started.")
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
                print("[RX] Received error 'E' from MCU.")
            elif b == b'\xF0':
                # Button frame prefix: expect a second byte
                btn = ser.read(1)
                if not btn:
                    continue
                if btn == b'P':
                    button_event_queue.put("play_pause")
                    print("[RX] Button: PLAY/PAUSE")
                elif btn == b'N':
                    button_event_queue.put("next")
                    print("[RX] Button: NEXT")
                elif btn == b'B':
                    button_event_queue.put("prev")
                    print("[RX] Button: PREV")
                else:
                    print(f"[RX] Unknown button code: {btn!r}")
            else:
                # Unknown/ignored byte
                pass
    finally:
        print("[RX] Thread exiting.")


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
    """
    Send a single packet (header + payload + CRC) and block until
    'H' then 'S'/'E' are received (via events).
    Returns True on success, False if MCU reports error or timeout.
    """
    pkt = build_packet(data_type, flags, seq, payload)
    header = pkt[:9]
    body   = pkt[9:]

    # Send header
    header_ack_event.clear()
    with TX_LOCK:
        ser.write(header)
        ser.flush()
    print(f"[TX] Header sent ({label}, len={len(header)})")

    # Wait for 'H'
    try:
        wait_header_ack(label=label)
    except TimeoutError as e:
        print(f"[TX] ERROR: {e}")
        return False

    # Send body
    with TX_LOCK:
        ser.write(body)
        ser.flush()
    print(f"[TX] Body sent   ({label}, len={len(body)})")

    # Wait for 'S' or 'E'
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
    print(f"Converting image '{image_path}' to {W}x{H} RGB565...")
    full_bytes = image_to_rgb565_be_bytes(image_path, W, H)
    print(f"Image converted: {len(full_bytes)} bytes total.")

    max_rows = compute_max_rows_per_packet(W)
    print(f"Max rows per packet at width {W}: {max_rows}")

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

    print(f"Prepared {len(packets)} image packets.")
    return packets


def make_audio_packets(wav_path: str) -> List[PacketEntry]:
    """
    Precompute all audio packets (seq, flags, payload, debug_label).
    Payload size is limited to AUDIO_MAX_PAYLOAD to match STM32.
    """
    print(f"Loading WAV file '{wav_path}'...")
    samples, fs = load_wav_mono_16bit(wav_path)
    print(f"WAV loaded: {len(samples)} samples @ {fs} Hz.")

    raw = samples.tobytes()
    total_bytes = len(raw)
    print(f"Total audio bytes: {total_bytes}")

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

    print(f"Prepared {len(packets)} audio packets.")
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

    print("\n[MODE] Interleaved Image + Audio")

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
    print("[INTERLEAVE] Sending first audio packet immediately.")
    if not send_packet_with_handshake(ser, PKT_DATA_AUDIO, flags, seq, payload, label):
        print("[MODE] Aborting interleaved transfer due to error on first audio packet.")
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
            print(f"[INTERLEAVE] MCU requested audio -> {label}")
            if not send_packet_with_handshake(ser, PKT_DATA_AUDIO, flags, seq, payload, label):
                print("[MODE] Aborting interleaved transfer due to audio error.")
                return
            aud_idx += 1
            did_something = True
            continue

        if img_idx < total_img:
            seq, flags, payload, label = img_packets[img_idx]
            print(f"[INTERLEAVE] Sending image tile -> {label}")
            if not send_packet_with_handshake(ser, PKT_DATA_IMAGE, flags, seq, payload, label):
                print("[MODE] Aborting interleaved transfer due to image error.")
                return
            img_idx += 1
            did_something = True
            continue

        if aud_idx < total_aud and not did_something:
            time.sleep(0.001)

    print("[MODE] Interleaved Image + Audio complete.")


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


# ---------- Jukebox mode (playlist + hardware buttons) ----------

def run_jukebox(ser: serial.Serial):
    """
    Jukebox mode:
      - Build a playlist of Tracks.
      - Use MCU buttons (next/prev/play-pause) to control playback.
      - Respond to 'A' requests to stream audio.
    """
    print("\n=== Jukebox mode ===")
    print("This mode will play a playlist and respond to hardware buttons.")
    print("Album art will be sent as 320x320 cover at top of the display.")
    print("Press Ctrl+C to exit jukebox mode and return to the main menu.\n")

    # ----- Build playlist -----
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

    queue_tracks: List[Track] = []
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
    player_state: str = "IDLE"  # "IDLE", "STARTING_TRACK", "PLAYING", "PAUSED"

    clear_all_events()

    def start_track(track: Track):
        nonlocal current_track, player_state
        print(f"[JUKEBOX] Starting track: {track.title} – {track.artist}")
        current_track = track
        current_track.audio_index = 0
        current_track.finished = False
        clear_all_events()
        display_track_ui(ser, current_track)

        # Send first audio packet immediately (no 'A' gating)
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
            # We do NOT send AUDIO_STOP here; we just stop feeding new audio.
        elif player_state == "PAUSED":
            print("[JUKEBOX] Resuming playback.")
            player_state = "PLAYING"
        else:
            print(f"[JUKEBOX] Play/pause pressed in state={player_state}; ignoring.")

    def send_next_audio_packet_for_current_track():
        nonlocal current_track
        if current_track is None:
            return
        if current_track.audio_index >= len(current_track.audio_packets):
            # No more packets
            return

        seq, flags, payload, label = current_track.audio_packets[current_track.audio_index]
        print(f"[JUKEBOX] Sending audio packet idx={current_track.audio_index} -> {label}")
        ok = send_packet_with_handshake(
            ser, PKT_DATA_AUDIO, flags, seq, payload, label, timeout=10.0
        )
        if not ok:
            print("[JUKEBOX] Error sending audio packet; stream may glitch.")
        if (flags & PKT_FLAG_LAST) != 0:
            print("[JUKEBOX] Last audio packet sent for this track.")
            current_track.finished = True
        current_track.audio_index += 1

    # ----- Main jukebox loop -----
    try:
        while True:
            # A. Start first track if idle and we have tracks
            if player_state == "IDLE" and current_track is None and queue_tracks:
                go_to_next_track()

            # B. Handle button events
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

            # C. Handle 'A' (next audio chunk) only when playing
            if player_state == "PLAYING" and current_track is not None:
                if audio_req_event.is_set():
                    audio_req_event.clear()
                    send_next_audio_packet_for_current_track()

            # D. End-of-track auto-advance
            if player_state == "PLAYING" and current_track is not None and current_track.finished:
                if queue_tracks:
                    print("[JUKEBOX] Track finished; advancing to next.")
                    go_to_next_track()
                else:
                    print("[JUKEBOX] Track finished; playlist done.")
                    current_track = None
                    player_state = "IDLE"

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[JUKEBOX] Exiting jukebox mode and returning to main menu.")
        # Let caller return to main menu
        return


# ---------- Main ----------

def main():
    global rx_thread_running

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
                        "Exit"
                    ]
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
    try:
        main()
    except KeyboardInterrupt:
        print("\nCanceled by user.")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
