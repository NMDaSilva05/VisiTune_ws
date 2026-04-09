#!/usr/bin/env python3
import sys
import os
import numpy as np
from pydub import AudioSegment


def convert_audio(input_file, out_prefix):
    print(f"\nProcessing: {input_file}")

    # Load audio
    audio = AudioSegment.from_file(input_file)

    # Mono, 24 kHz, 16-bit PCM
    audio = audio.set_channels(1)
    audio = audio.set_frame_rate(24000)
    audio = audio.set_sample_width(2)

    # Output WAV
    wav_path = out_prefix + ".wav"
    audio.export(wav_path, format="wav")
    print("  WAV →", wav_path)

    # Raw PCM bytes
    raw_data = audio.raw_data
    raw_path = out_prefix + ".raw"
    with open(raw_path, "wb") as f:
        f.write(raw_data)
    print("  RAW →", raw_path)

    # Convert raw → int16 array
    samples = np.frombuffer(raw_data, dtype=np.int16)

    # Convert 16-bit signed → 12-bit unsigned right aligned (STM32 DAC format)
    samples_12 = ((samples.astype(np.int32) + 32768) >> 4).clip(0, 4095).astype(np.uint16)

    # Output C header
    h_path = out_prefix + ".h"
    name = os.path.basename(out_prefix)

    with open(h_path, "w") as f:
        f.write("// Auto-generated audio header\n\n")
        guard = f"AUDIO_DATA_{name.upper()}_H"
        f.write(f"#ifndef {guard}\n#define {guard}\n\n")
        f.write("#include <stdint.h>\n\n")

        f.write(f"const uint16_t {name}[] = {{\n")
        for i, v in enumerate(samples_12):
            sep = "," if i < len(samples_12)-1 else ""
            f.write(f"  {v}{sep}\n")
        f.write("};\n\n")

        f.write(f"const uint32_t {name}_len = {len(samples_12)};\n\n")
        f.write(f"#endif // {guard}\n")

    print("  Header →", h_path)


def batch_convert(directory):
    print(f"Scanning directory: {directory}")

    supported = (".wav", ".mp3", ".flac", ".ogg", ".m4a", ".aac")

    for fname in os.listdir(directory):
        path = os.path.join(directory, fname)

        if not os.path.isfile(path):
            continue

        if not fname.lower().endswith(supported):
            print("Skipping (not audio):", fname)
            continue

        base, _ = os.path.splitext(fname)
        out_prefix = os.path.join(directory, base)

        convert_audio(path, out_prefix)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 batch_audio_to_c.py <directory>")
        sys.exit(1)

    batch_convert(sys.argv[1])
