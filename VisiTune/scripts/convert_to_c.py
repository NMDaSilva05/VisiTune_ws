#!/usr/bin/env python3
import sys
import os
import re
import numpy as np
from pydub import AudioSegment


def sanitize_name(text):
    text = text.lower()
    text = re.sub(r'[^a-z0-9]+', '_', text)
    text = re.sub(r'_+', '_', text)
    if not text[0].isalpha():
        text = "snd_" + text
    return text


def process_audio_file(path):
    print(f"Processing: {path}")
    audio = AudioSegment.from_file(path)

    # Convert: mono, 24 kHz, 16-bit
    audio = audio.set_channels(1)
    audio = audio.set_frame_rate(24000)
    audio = audio.set_sample_width(2)

    # Get raw PCM bytes
    raw = audio.raw_data
    samples = np.frombuffer(raw, dtype=np.int16)

    # Convert to 12-bit unsigned right-aligned
    samples_12 = ((samples.astype(np.int32) + 32768) >> 4).clip(0, 4095).astype(np.uint16)

    return samples_12


def batch_to_single_header(directory, header_name="audio_bank.h"):
    supported = (".wav", ".mp3", ".flac", ".ogg", ".m4a", ".aac")

    print(f"Scanning directory: {directory}")

    entries = []   # list of (symbol_name, samples)

    for fname in os.listdir(directory):
        path = os.path.join(directory, fname)

        if not os.path.isfile(path):
            continue

        if not fname.lower().endswith(supported):
            print("Skipping:", fname)
            continue

        base, _ = os.path.splitext(fname)
        symbol = sanitize_name(base)
        samples = process_audio_file(path)

        entries.append((symbol, samples))

    # Write header
    out_path = os.path.join(directory, header_name)
    print("\nWriting header:", out_path)

    with open(out_path, "w") as f:
        f.write("// Auto-generated audio bank header\n\n")
        guard = "AUDIO_BANK_H"
        f.write(f"#ifndef {guard}\n#define {guard}\n\n")
        f.write("#include <stdint.h>\n\n")

        for symbol, samples in entries:
            f.write(f"// ---- {symbol} ----\n")
            f.write(f"const uint16_t {symbol}[] = {{\n")
            for i, v in enumerate(samples):
                sep = "," if i < len(samples)-1 else ""
                f.write(f"  {v}{sep}\n")
            f.write(f"}};\n\n")
            f.write(f"const uint32_t {symbol}_len = {len(samples)};\n\n")

        f.write(f"#endif // {guard}\n")

    print("DONE!")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 batch_audio_to_one_header.py <directory>")
        sys.exit(1)

    batch_to_single_header(sys.argv[1])
