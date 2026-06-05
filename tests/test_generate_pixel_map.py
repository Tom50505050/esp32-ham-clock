"""Tests for generate_pixel_map.py — pixel-art world map BMP generator.

Covers:
- BMP header structure (signature, dimensions, bpp)
- File size consistency
- Non-trivial pixel content (not all-zero)
- Multiple output sizes
"""

import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from generate_pixel_map import create_pixel_world_map


class TestCreatePixelWorldMap:
    def test_creates_file(self, tmp_dir):
        path = str(tmp_dir / "pixel.bmp")
        create_pixel_world_map(20, 10, path)
        assert os.path.isfile(path)

    def test_bm_signature(self, tmp_dir):
        path = str(tmp_dir / "pixel.bmp")
        create_pixel_world_map(20, 10, path)
        with open(path, "rb") as f:
            assert f.read(2) == b"BM"

    def test_dimensions_in_header(self, tmp_dir):
        w, h = 30, 20
        path = str(tmp_dir / "pixel.bmp")
        create_pixel_world_map(w, h, path)
        with open(path, "rb") as f:
            data = f.read(14 + 40)
        sw = struct.unpack_from("<i", data, 18)[0]
        sh = struct.unpack_from("<i", data, 22)[0]
        assert (sw, sh) == (w, h)

    def test_bpp_is_16(self, tmp_dir):
        path = str(tmp_dir / "pixel.bmp")
        create_pixel_world_map(20, 10, path)
        with open(path, "rb") as f:
            data = f.read(14 + 40)
        assert struct.unpack_from("<H", data, 28)[0] == 16

    def test_file_size_matches_header(self, tmp_dir):
        path = str(tmp_dir / "pixel.bmp")
        create_pixel_world_map(40, 30, path)
        actual = os.path.getsize(path)
        with open(path, "rb") as f:
            header_size = struct.unpack_from("<I", f.read(14), 2)[0]
        assert actual == header_size

    def test_pixel_data_not_all_zero(self, tmp_dir):
        """The map should draw continents, so pixel data should not be blank."""
        path = str(tmp_dir / "pixel.bmp")
        create_pixel_world_map(140, 80, path)
        with open(path, "rb") as f:
            f.seek(54)
            pixel_data = f.read()
        assert any(b != 0 for b in pixel_data)

    def test_different_sizes(self, tmp_dir):
        for w, h in [(10, 5), (140, 80), (320, 180)]:
            path = str(tmp_dir / f"map_{w}x{h}.bmp")
            create_pixel_world_map(w, h, path)
            assert os.path.isfile(path)
            with open(path, "rb") as f:
                data = f.read(14 + 40)
            sw = struct.unpack_from("<i", data, 18)[0]
            sh = struct.unpack_from("<i", data, 22)[0]
            assert (sw, sh) == (w, h)

    def test_pixel_data_has_variety(self, tmp_dir):
        """With continents and ocean, there should be multiple distinct pixel values."""
        path = str(tmp_dir / "pixel.bmp")
        create_pixel_world_map(140, 80, path)
        with open(path, "rb") as f:
            f.seek(54)
            data = f.read()
        unique_words = set()
        for i in range(0, len(data) - 1, 2):
            unique_words.add(struct.unpack_from("<H", data, i)[0])
        # Ocean + continent colours → at least 2 distinct values
        assert len(unique_words) >= 2
