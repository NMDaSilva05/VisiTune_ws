import os
import sys
import wave
import serial
from serial.tools import list_ports

def choose_from_list(prompt, options):
    """
    Show a numbered list and return the chosen value.
    """
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
    options = [f"{p.device} — {p.description}" for p in ports]
    if ports:
        selected = choose_from_list("Select a COM port:", options)
        if selected in options:
            # extract device (left side before ' — ')
            return selected.split(" — ", 1)[0]
        return selected
    else:
        print("No serial ports found.")
        sys.exit(1)
        

def prompt_baud():
    baud_options = ["115200", "460800"]
    selected = choose_from_list("Select a baud rate:", baud_options)
    return int(selected)

def prompt_chunk_size():
    while True:
        s = input("Chunk size in bytes (0 = send whole file in one go): ").strip()
        if s.isdigit():
            return int(s)
        print("Please enter a non-negative integer (e.g., 0, 512, 1024).")

def prompt_wav_file():
    wavs = [f for f in os.listdir(".") if f.lower().endswith(".wav")]
    wavs.sort()
    if wavs:
        return choose_from_list("Select a WAV file from the current folder:", wavs)
    else:
        print("No .wav files found in the current directory.")
        sys.exit(1)

def print_status(packets, total_bytes):
    # Refresh a single line with current totals
    sys.stdout.write(f"\rPackets: {packets:,}  |  Bytes: {total_bytes:,}")
    sys.stdout.flush()

def send_entire_file(ser, wav):
    # Read all frames, write once
    nframes = wav.getnframes()
    data = wav.readframes(nframes)
    print(f"Sending entire file: {len(data)} bytes")
    ser.write(data)
    ser.flush()
    print("Done.")

def send_in_chunks_with_ack(ser, wav, chunk_size):
    # Send first chunk immediately
    chunk = wav.readframes(chunk_size)
    
    total_sent = 0
    packets = 0
    
    if chunk:
        ser.write(chunk)
        ser.flush()
        packets += 1
        total_sent += len(chunk)
        print_status(packets, total_sent)

    # Then: wait for 'S' before each subsequent chunk
    while True:
        chunk = wav.readframes(chunk_size)
        if not chunk:
            break

        # Wait until we receive a single ‘S’ (0x53) from the other side
        while ser.read(1) != b'S':
            # ser.timeout controls how long read() waits each loop iteration
            pass

        ser.write(chunk)
        ser.flush()
        packets += 1
        total_sent += len(chunk)
        print_status(packets, total_sent)

    sys.stdout.write("\n")  # finish the status line
    print("All chunks sent.")

def main():
    port = prompt_port()
    baud = prompt_baud()
    chunk_size = prompt_chunk_size()
    wav_path = prompt_wav_file()

    print("\n=== Configuration ===")
    print(f"Port      : {port}")
    print(f"Baud      : {baud}")
    print(f"Chunk size: {chunk_size} (0 means one-shot)")
    print(f"WAV file  : {wav_path}")
    print("=====================\n")

    # Open serial port and WAV file
    with serial.Serial(port, baud, timeout=1) as ser, wave.open(wav_path, 'rb') as wav:
        # Clear any stale incoming data before starting the handshake
        ser.reset_input_buffer()

        if chunk_size == 0:
            send_entire_file(ser, wav)
        else:
            send_in_chunks_with_ack(ser, wav, chunk_size)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCanceled by user.")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

