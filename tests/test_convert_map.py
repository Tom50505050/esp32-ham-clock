"""Tests for convert_map.py — BMP 24-bit → 16-bit RGB565 converter.

convert_map.py is nearly identical to convert_bmp.py but with minor
output differences.  These tests verify the same structural guarantees.
"""

import os
import struct
import sys

import pytest
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from convert_map import convert_bmp_to_rgb565


def _header(path):
    with open(path, "rb") as f:
        return f.read(14 + 40)


class TestConvertMapRgb565:
    def test_bm_signature(self, sample_rgb_image, tmp_dir):
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        with open(out, "rb") as f:
            assert f.read(2) == b"BM"

    def test_dimensions_preserved(self, sample_rgb_image, tmp_dir):
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        h = _header(out)
        assert struct.unpack_from("<i", h, 18)[0] == 4
        assert struct.unpack_from("<i", h, 22)[0] == 4

    def test_16bit_output(self, sample_rgb_image, tmp_dir):
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        h = _header(out)
        assert struct.unpack_from("<H", h, 28)[0] == 16

    def test_file_size_consistent(self, sample_rgb_image, tmp_dir):
        out = str(tmp_dir / "out.bmp")
        convert_bmp_to_rgb565(sample_rgb_image, out)
        actual = os.path.getsize(out)
        h = _header(out)
        expected = struct.unpack_from("<I", h, 2)[0]
        assert actual == expected

    def test_red_pixel(self, tmp_dir):
        img = Image.new("RGB", (1, 1), (255, 0, 0))
        inp = str(tmp_dir / "r.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "r_out.bmp")
        convert_bmp_to_rgb565(inp, out)
        with open(out, "rb") as f:
            f.seek(54)
            assert struct.unpack("<H", f.read(2))[0] == 0xF800

    def test_green_pixel(self, tmp_dir):
        img = Image.new("RGB", (1, 1), (0, 255, 0))
        inp = str(tmp_dir / "g.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "g_out.bmp")
        convert_bmp_to_rgb565(inp, out)
        with open(out, "rb") as f:
            f.seek(54)
            assert struct.unpack("<H", f.read(2))[0] == 0x07E0

    def test_blue_pixel(self, tmp_dir):
        img = Image.new("RGB", (1, 1), (0, 0, 255))
        inp = str(tmp_dir / "b.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "b_out.bmp")
        convert_bmp_to_rgb565(inp, out)
        with open(out, "rb") as f:
            f.seek(54)
            assert struct.unpack("<H", f.read(2))[0] == 0x001F

    def test_odd_width_padding(self, tmp_dir):
        img = Image.new("RGB", (5, 3), (0, 0, 0))
        inp = str(tmp_dir / "5x3.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "5x3_out.bmp")
        convert_bmp_to_rgb565(inp, out)

        row_bytes = 5 * 2  # 10
        row_size = (row_bytes + 3) & ~3  # 12
        expected = 14 + 40 + row_size * 3
        assert os.path.getsize(out) == expected

    def test_various_sizes(self, tmp_dir):
        for w, h in [(1, 1), (7, 3), (16, 16)]:
            img = Image.new("RGB", (w, h), (50, 100, 150))
            inp = str(tmp_dir / f"{w}x{h}.bmp")
            img.save(inp, "BMP")
            out = str(tmp_dir / f"{w}x{h}_out.bmp")
            convert_bmp_to_rgb565(inp, out)
            assert os.path.isfile(out)
