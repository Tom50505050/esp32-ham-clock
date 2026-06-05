"""Tests for convert_bmp.py — 24-bit BMP → 16-bit RGB565 converter.

Covers:
- BMP header structure (signature, dimensions, bpp, data offset)
- RGB565 pixel encoding correctness
- Row padding / alignment
- File size consistency
"""

import os
import struct
import sys

import pytest
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from convert_bmp import convert_bmp_to_rgb565


def _read_header(path):
    with open(path, "rb") as f:
        return f.read(14 + 40)


class TestConvertBmpToRgb565:
    # --- header correctness -------------------------------------------------

    def test_output_has_bm_signature(self, sample_rgb_image, tmp_dir):
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        with open(out, "rb") as f:
            assert f.read(2) == b"BM"

    def test_output_dimensions_match_input(self, sample_rgb_image, tmp_dir):
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        hdr = _read_header(out)
        w = struct.unpack_from("<i", hdr, 18)[0]
        h = struct.unpack_from("<i", hdr, 22)[0]
        assert (w, h) == (4, 4)

    def test_output_bpp_is_16(self, sample_rgb_image, tmp_dir):
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        hdr = _read_header(out)
        bpp = struct.unpack_from("<H", hdr, 28)[0]
        assert bpp == 16

    def test_data_offset_is_54(self, sample_rgb_image, tmp_dir):
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        hdr = _read_header(out)
        offset = struct.unpack_from("<I", hdr, 10)[0]
        assert offset == 54  # 14 + 40

    def test_dib_header_size_is_40(self, sample_rgb_image, tmp_dir):
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        hdr = _read_header(out)
        dib_size = struct.unpack_from("<I", hdr, 14)[0]
        assert dib_size == 40

    # --- file size ----------------------------------------------------------

    def test_file_size_header_matches_actual(self, sample_rgb_image, tmp_dir):
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        actual = os.path.getsize(out)
        hdr = _read_header(out)
        header_size = struct.unpack_from("<I", hdr, 2)[0]
        assert actual == header_size

    def test_file_size_formula(self, sample_rgb_image, tmp_dir):
        """Verify size = 14 + 40 + rows * row_size (with padding)."""
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        w, h = 4, 4
        row_bytes = w * 2
        row_size = (row_bytes + 3) & ~3  # align to 4
        expected = 14 + 40 + row_size * h
        assert os.path.getsize(out) == expected

    # --- pixel encoding -----------------------------------------------------

    def test_pure_red_pixel_encoding(self, tmp_dir):
        img = Image.new("RGB", (1, 1), (255, 0, 0))
        inp = str(tmp_dir / "red.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "red_out.bmp")
        convert_bmp_to_rgb565(inp, out)

        with open(out, "rb") as f:
            f.seek(54)
            pixel = struct.unpack("<H", f.read(2))[0]

        # RGB565: R=0b11111, G=0b000000, B=0b00000 → 0xF800
        assert pixel == 0xF800

    def test_pure_green_pixel_encoding(self, tmp_dir):
        img = Image.new("RGB", (1, 1), (0, 255, 0))
        inp = str(tmp_dir / "green.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "green_out.bmp")
        convert_bmp_to_rgb565(inp, out)

        with open(out, "rb") as f:
            f.seek(54)
            pixel = struct.unpack("<H", f.read(2))[0]

        assert pixel == 0x07E0

    def test_pure_blue_pixel_encoding(self, tmp_dir):
        img = Image.new("RGB", (1, 1), (0, 0, 255))
        inp = str(tmp_dir / "blue.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "blue_out.bmp")
        convert_bmp_to_rgb565(inp, out)

        with open(out, "rb") as f:
            f.seek(54)
            pixel = struct.unpack("<H", f.read(2))[0]

        assert pixel == 0x001F

    def test_white_pixel_encoding(self, tmp_dir):
        img = Image.new("RGB", (1, 1), (255, 255, 255))
        inp = str(tmp_dir / "white.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "white_out.bmp")
        convert_bmp_to_rgb565(inp, out)

        with open(out, "rb") as f:
            f.seek(54)
            pixel = struct.unpack("<H", f.read(2))[0]

        assert pixel == 0xFFFF

    def test_black_pixel_encoding(self, tmp_dir):
        img = Image.new("RGB", (1, 1), (0, 0, 0))
        inp = str(tmp_dir / "black.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "black_out.bmp")
        convert_bmp_to_rgb565(inp, out)

        with open(out, "rb") as f:
            f.seek(54)
            pixel = struct.unpack("<H", f.read(2))[0]

        assert pixel == 0x0000

    # --- row padding --------------------------------------------------------

    def test_odd_width_padding(self, tmp_dir):
        """3-pixel wide → 6 bytes/row → padded to 8 bytes."""
        img = Image.new("RGB", (3, 2), (255, 0, 0))
        inp = str(tmp_dir / "odd.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "odd_out.bmp")
        convert_bmp_to_rgb565(inp, out)

        row_bytes = 3 * 2  # 6
        row_size = (row_bytes + 3) & ~3  # 8
        expected_size = 14 + 40 + row_size * 2
        assert os.path.getsize(out) == expected_size

    # --- various image sizes ------------------------------------------------

    def test_non_square_image(self, tmp_dir):
        img = Image.new("RGB", (10, 5), (100, 200, 50))
        inp = str(tmp_dir / "rect.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "rect_out.bmp")
        convert_bmp_to_rgb565(inp, out)

        hdr = _read_header(out)
        w = struct.unpack_from("<i", hdr, 18)[0]
        h = struct.unpack_from("<i", hdr, 22)[0]
        assert (w, h) == (10, 5)

    def test_single_pixel(self, tmp_dir):
        img = Image.new("RGB", (1, 1), (0, 128, 0))
        inp = str(tmp_dir / "1px.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "1px_out.bmp")
        convert_bmp_to_rgb565(inp, out)
        assert os.path.isfile(out)
        assert os.path.getsize(out) > 54
