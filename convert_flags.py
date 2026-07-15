#!/usr/bin/env python3
"""Convert country flag PNGs to 16-bit BMP (RGB565) for TFT_eSPI on ESP32.

Usage:
  1. Download flag PNGs (e.g. flagcdn.com/w20.zip) and extract to flags_png/
  2. Run: python convert_flags.py
  3. Output BMPs go to littlefs_data/flags/
  4. Run mklittlefs to rebuild LittleFS image
"""

import os
import struct
from PIL import Image

FLAG_W = 24
FLAG_H = 16

# Paths relative to this script's directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(SCRIPT_DIR, "flags_png")
dst_dir = os.path.join(SCRIPT_DIR, "littlefs_data", "flags")


def rgb_to_565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def save_bmp_16bit(filepath, img):
    """Save an image as 16-bit BMP (RGB565, little-endian byte order for TFT_eSPI)."""
    w, h = img.size
    row_bytes = w * 2
    # BMP rows must be padded to 4-byte boundary
    pad = (4 - (row_bytes % 4)) % 4
    row_size = row_bytes + pad
    pixel_data_size = row_size * h
    file_size = 14 + 40 + pixel_data_size

    pixels = img.load()

    with open(filepath, 'wb') as f:
        # BMP File Header (14 bytes)
        f.write(b'BM')
        f.write(struct.pack('<I', file_size))   # file size
        f.write(struct.pack('<HH', 0, 0))       # reserved
        f.write(struct.pack('<I', 14 + 40))     # offset to pixel data

        # DIB Header - BITMAPINFOHEADER (40 bytes)
        f.write(struct.pack('<I', 40))          # header size
        f.write(struct.pack('<i', w))           # width
        f.write(struct.pack('<i', -h))          # height (negative = top-down)
        f.write(struct.pack('<HH', 1, 16))      # planes=1, bpp=16
        f.write(struct.pack('<I', 0))           # compression=BI_RGB
        f.write(struct.pack('<I', pixel_data_size))
        f.write(struct.pack('<i', 2835))        # X pixels per meter (72 dpi)
        f.write(struct.pack('<i', 2835))        # Y pixels per meter
        f.write(struct.pack('<I', 0))           # colors in table
        f.write(struct.pack('<I', 0))           # important colors

        # Pixel data (top-down, RGB565 little-endian)
        for y in range(h):
            for x in range(w):
                r, g, b = pixels[x, y][:3]
                val = rgb_to_565(r, g, b)
                f.write(struct.pack('<H', val))
            # Row padding
            f.write(b'\x00' * pad)


if not os.path.isdir(src_dir):
    print(f"ERROR: Source directory not found: {src_dir}")
    print(f"Download flag PNGs and extract them to: {src_dir}")
    exit(1)

os.makedirs(dst_dir, exist_ok=True)

files = [f for f in os.listdir(src_dir) if f.endswith('.png')]
print(f"Converting {len(files)} flags to 16-bit BMP ({FLAG_W}x{FLAG_H})...")
print(f"  Source:      {src_dir}")
print(f"  Destination: {dst_dir}")

converted = 0
for fname in sorted(files):
    code = fname.replace('.png', '').upper()
    src = os.path.join(src_dir, fname)
    dst = os.path.join(dst_dir, f"{code}.bmp")
    try:
        img = Image.open(src).convert('RGB')
        img = img.resize((FLAG_W, FLAG_H), Image.LANCZOS)
        save_bmp_16bit(dst, img)
        converted += 1
    except Exception as e:
        print(f"  SKIP {fname}: {e}")

print(f"Done: {converted}/{len(files)} flags converted")
print(f"Output: {dst_dir}")

# Calculate total size
total = sum(os.path.getsize(os.path.join(dst_dir, f)) for f in os.listdir(dst_dir) if f.endswith('.bmp'))
print(f"Total flags size: {total / 1024:.1f} KB ({len([f for f in os.listdir(dst_dir) if f.endswith('.bmp')])} files)")
