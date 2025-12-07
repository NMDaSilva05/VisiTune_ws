import wave
import struct
import sys

def wav_to_raw_16bit(wav_in, raw_out):
    w = wave.open(wav_in, 'rb')

    channels = w.getnchannels()
    sampwidth = w.getsampwidth()
    framerate = w.getframerate()
    nframes = w.getnframes()

    print("Channels:", channels)
    print("Sample width:", sampwidth)
    print("Sample rate:", framerate, "Hz")
    print("Frames:", nframes)

    if sampwidth != 2:
        print("ERROR: This script only supports 16-bit PCM WAV files.")
        sys.exit(1)

    if framerate != 24000:
        print("WARNING: WAV is not 24 kHz! Converting anyway...")

    out = open(raw_out, "wb")

    for i in range(nframes):
        frame = w.readframes(1)

        if channels == 1:
            # 16-bit signed sample (keep as-is)
            sample = struct.unpack("<h", frame)[0]
        else:
            # Stereo → mono by averaging L and R
            L, R = struct.unpack("<hh", frame)
            sample = (L + R) // 2

        # Write raw 16-bit signed value
        out.write(struct.pack("<h", sample))

    out.close()
    w.close()

    print("Wrote:", raw_out)


# Example:
in_file = input("input WAV file: ")
out_file = input("output RAW file: ")
wav_to_raw_16bit(in_file, out_file)
