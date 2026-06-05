"""Tests for resize_map.py — BMP resize to 140x80 + RGB565 conversion.

Covers:
- Output file creation
- BMP header correctness (dimensions = 140×80)
- 16-bit depth
- File size consistency
- Pixel data encoding
"""

import os
import struct
import sys

import pytest
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from resize_map import resize_bmp_to_140x80


def _header(path):
    with open(path, "rb") as f:
        return f.read(14 + 40)


class TestResizeBmpTo140x80:
    def test_creates_output_file(self, sample_large_image, tmp_dir):
        out = str(tmp_dir / "resized.bmp")
        resize_bmp_to_140x80(sample_large_image, out)
        assert os.path.isfile(out)

    def test_bm_signature(self, sample_large_image, tmp_dir):
        out = str(tmp_dir / "resized.bmp")
        resize_bmp_to_140x80(sample_large_image, out)
        with open(out, "rb") as f:
            assert f.read(2) == b"BM"

    def test_dimensions_are_140x80(self, sample_large_image, tmp_dir):
        out = str(tmp_dir / "resized.bmp")
        resize_bmp_to_140x80(sample_large_image, out)
        h = _header(out)
        w = struct.unpack_from("<i", h, 18)[0]
        ht = struct.unpack_from("<i", h, 22)[0]
        assert (w, ht) == (140, 80)

    def test_bpp_is_16(self, sample_large_image, tmp_dir):
        out = str(tmp_dir / "resized.bmp")
        resize_bmp_to_140x80(sample_large_image, out)
        h = _header(out)
        assert struct.unpack_from("<H", h, 28)[0] == 16

    def test_file_size_consistent(self, sample_large_image, tmp_dir):
        out = str(tmp_dir / "resized.bmp")
        resize_bmp_to_140x80(sample_large_image, out)
        actual = os.path.getsize(out)
        h = _header(out)
        expected = struct.unpack_from("<I", h, 2)[0]
        assert actual == expected

    def test_file_size_formula(self, sample_large_image, tmp_dir):
        out = str(tmp_dir / "resized.bmp")
        resize_bmp_to_140x80(sample_large_image, out)
        w, ht = 140, 80
        row_bytes = w * 2
        row_size = (row_bytes + 3) & ~3
        expected = 14 + 40 + row_size * ht
        assert os.path.getsize(out) == expected

    def test_accepts_small_input(self, tmp_dir):
        """Even an image smaller than 140x80 should work."""
        img = Image.new("RGB", (10, 5), (200, 100, 50))
        inp = str(tmp_dir / "small.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "small_out.bmp")
        resize_bmp_to_140x80(inp, out)

        h = _header(out)
        w = struct.unpack_from("<i", h, 18)[0]
        ht = struct.unpack_from("<i", h, 22)[0]
        assert (w, ht) == (140, 80)

    def test_accepts_square_input(self, tmp_dir):
        img = Image.new("RGB", (300, 300), (10, 20, 30))
        inp = str(tmp_dir / "sq.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "sq_out.bmp")
        resize_bmp_to_140x80(inp, out)

        h = _header(out)
        w = struct.unpack_from("<i", h, 18)[0]
        ht = struct.unpack_from("<i", h, 22)[0]
        assert (w, ht) == (140, 80)

    def test_uniform_colour_preserved_approximately(self, tmp_dir):
        """A solid-red image should still be mostly red after resize."""
        img = Image.new("RGB", (280, 160), (255, 0, 0))
        inp = str(tmp_dir / "red.bmp")
        img.save(inp, "BMP")
        out = str(tmp_dir / "red_out.bmp")
        resize_bmp_to_140x80(inp, out)

        with open(out, "rb") as f:
            f.seek(54)
            pixel = struct.unpack("<H", f.read(2))[0]

        # Pure red in RGB565 = 0xF800
        assert pixel == 0xF800
