"""Tests for generate_map_simple.py — the pure-Python world map generator.

Covers:
- rgb888_to_rgb565 colour conversion
- lon_lat_to_xy coordinate projection
- point_in_polygon ray-casting algorithm
- create_world_map_bmp end-to-end BMP output
"""

import os
import struct
import sys

import pytest

# Ensure repo root is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from generate_map_simple import create_world_map_bmp


# ---------------------------------------------------------------------------
# Helpers — re-implement the inner functions so they can be tested in
# isolation (they are local to create_world_map_bmp in the source).
# ---------------------------------------------------------------------------

def rgb888_to_rgb565(r, g, b):
    r5 = (r >> 3) & 0x1F
    g6 = (g >> 2) & 0x3F
    b5 = (b >> 3) & 0x1F
    return (r5 << 11) | (g6 << 5) | b5


def lon_lat_to_xy(lon, lat, w, h):
    x = int((lon + 180) * w / 360)
    y = int((90 - lat) * h / 180)
    return (max(0, min(w - 1, x)), max(0, min(h - 1, y)))


def point_in_polygon(x, y, polygon):
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


# ===== rgb888_to_rgb565 ====================================================

class TestRgb888ToRgb565:
    def test_black(self):
        assert rgb888_to_rgb565(0, 0, 0) == 0x0000

    def test_white(self):
        # 5-bit R=31, 6-bit G=63, 5-bit B=31 → 0xFFFF
        assert rgb888_to_rgb565(255, 255, 255) == 0xFFFF

    def test_pure_red(self):
        # R=31 in top 5 bits → 0xF800
        assert rgb888_to_rgb565(255, 0, 0) == 0xF800

    def test_pure_green(self):
        # G=63 in middle 6 bits → 0x07E0
        assert rgb888_to_rgb565(0, 255, 0) == 0x07E0

    def test_pure_blue(self):
        # B=31 in bottom 5 bits → 0x001F
        assert rgb888_to_rgb565(0, 0, 255) == 0x001F

    def test_mid_grey(self):
        val = rgb888_to_rgb565(128, 128, 128)
        r5 = (128 >> 3) & 0x1F  # 16
        g6 = (128 >> 2) & 0x3F  # 32
        b5 = (128 >> 3) & 0x1F  # 16
        expected = (r5 << 11) | (g6 << 5) | b5
        assert val == expected

    def test_result_fits_16_bits(self):
        for r in (0, 127, 255):
            for g in (0, 127, 255):
                for b in (0, 127, 255):
                    val = rgb888_to_rgb565(r, g, b)
                    assert 0 <= val <= 0xFFFF

    def test_quantisation_round_trip_lossy(self):
        """Lower bits are dropped, so round-trip is not exact."""
        original = (0b10101010, 0b01010101, 0b11001100)
        val = rgb888_to_rgb565(*original)
        r_out = ((val >> 11) & 0x1F) << 3
        g_out = ((val >> 5) & 0x3F) << 2
        b_out = (val & 0x1F) << 3
        # Top bits must match after quantisation
        assert abs(r_out - original[0]) < 8
        assert abs(g_out - original[1]) < 4
        assert abs(b_out - original[2]) < 8


# ===== lon_lat_to_xy ======================================================

class TestLonLatToXy:
    def test_origin(self):
        x, y = lon_lat_to_xy(0, 0, 360, 180)
        assert x == 180
        assert y == 90

    def test_top_left(self):
        x, y = lon_lat_to_xy(-180, 90, 360, 180)
        assert x == 0
        assert y == 0

    def test_bottom_right(self):
        x, y = lon_lat_to_xy(180, -90, 360, 180)
        # x = int(360*360/360) = 360 → clamped to 359
        assert x == 359
        assert y == 179

    def test_clamp_negative_coords(self):
        # Very far west/north should clamp to 0
        x, y = lon_lat_to_xy(-999, 999, 100, 100)
        assert x == 0
        assert y == 0

    def test_small_map(self):
        x, y = lon_lat_to_xy(0, 0, 10, 10)
        assert 0 <= x < 10
        assert 0 <= y < 10

    def test_proportional_scaling(self):
        x1, _ = lon_lat_to_xy(-90, 0, 360, 180)
        x2, _ = lon_lat_to_xy(90, 0, 360, 180)
        assert x2 - x1 == 180  # half the width


# ===== point_in_polygon ====================================================

class TestPointInPolygon:
    SQUARE = [(0, 0), (10, 0), (10, 10), (0, 10)]
    TRIANGLE = [(0, 0), (10, 0), (5, 10)]

    def test_inside_square(self):
        assert point_in_polygon(5, 5, self.SQUARE) is True

    def test_outside_square(self):
        assert point_in_polygon(15, 5, self.SQUARE) is False

    def test_inside_triangle(self):
        assert point_in_polygon(5, 3, self.TRIANGLE) is True

    def test_outside_triangle(self):
        assert point_in_polygon(9, 9, self.TRIANGLE) is False

    def test_far_away(self):
        assert point_in_polygon(100, 100, self.SQUARE) is False

    def test_negative_coords(self):
        polygon = [(-10, -10), (10, -10), (10, 10), (-10, 10)]
        assert point_in_polygon(0, 0, polygon) is True
        assert point_in_polygon(-5, -5, polygon) is True
        assert point_in_polygon(20, 20, polygon) is False

    def test_concave_polygon(self):
        # L-shaped polygon
        poly = [(0, 0), (10, 0), (10, 5), (5, 5), (5, 10), (0, 10)]
        assert point_in_polygon(2, 2, poly) is True   # inside lower-left
        assert point_in_polygon(2, 8, poly) is True    # inside upper-left
        assert point_in_polygon(8, 2, poly) is True    # inside lower-right arm
        assert point_in_polygon(8, 8, poly) is False   # outside upper-right notch


# ===== create_world_map_bmp (end-to-end) ===================================

class TestCreateWorldMapBmp:
    def test_creates_file(self, tmp_dir):
        path = str(tmp_dir / "map.bmp")
        create_world_map_bmp(20, 10, path)
        assert os.path.isfile(path)

    def test_bmp_signature(self, tmp_dir):
        path = str(tmp_dir / "map.bmp")
        create_world_map_bmp(20, 10, path)
        with open(path, "rb") as f:
            assert f.read(2) == b"BM"

    def test_bmp_dimensions_in_header(self, tmp_dir):
        w, h = 30, 20
        path = str(tmp_dir / "map.bmp")
        create_world_map_bmp(w, h, path)
        with open(path, "rb") as f:
            data = f.read(14 + 40)
        stored_w = struct.unpack_from("<i", data, 18)[0]
        stored_h = struct.unpack_from("<i", data, 22)[0]
        assert stored_w == w
        assert stored_h == h

    def test_bmp_bpp_is_16(self, tmp_dir):
        path = str(tmp_dir / "map.bmp")
        create_world_map_bmp(20, 10, path)
        with open(path, "rb") as f:
            data = f.read(14 + 40)
        bpp = struct.unpack_from("<H", data, 28)[0]
        assert bpp == 16

    def test_bmp_compression_bitfields(self, tmp_dir):
        path = str(tmp_dir / "map.bmp")
        create_world_map_bmp(20, 10, path)
        with open(path, "rb") as f:
            data = f.read(14 + 40)
        compression = struct.unpack_from("<I", data, 30)[0]
        assert compression == 3  # BI_BITFIELDS

    def test_bmp_rgb565_masks(self, tmp_dir):
        path = str(tmp_dir / "map.bmp")
        create_world_map_bmp(20, 10, path)
        with open(path, "rb") as f:
            f.seek(14 + 40)
            masks = struct.unpack("<III", f.read(12))
        assert masks == (0xF800, 0x07E0, 0x001F)

    def test_file_size_consistency(self, tmp_dir):
        w, h = 25, 15
        path = str(tmp_dir / "map.bmp")
        create_world_map_bmp(w, h, path)

        actual_size = os.path.getsize(path)

        with open(path, "rb") as f:
            header_file_size = struct.unpack_from("<I", f.read(14), 2)[0]

        assert actual_size == header_file_size

    def test_different_sizes(self, tmp_dir):
        for w, h in [(10, 5), (140, 80), (50, 30)]:
            path = str(tmp_dir / f"map_{w}x{h}.bmp")
            create_world_map_bmp(w, h, path)
            assert os.path.isfile(path)
            assert os.path.getsize(path) > 0
