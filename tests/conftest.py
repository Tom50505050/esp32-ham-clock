"""Shared fixtures for ESP32-HAM-CLOCK tests."""

import os
import tempfile

import pytest
from PIL import Image


@pytest.fixture
def tmp_dir(tmp_path):
    """Return a temporary directory path (pathlib.Path)."""
    return tmp_path


@pytest.fixture
def sample_rgb_image(tmp_path):
    """Create a small 4x4 24-bit RGB BMP for conversion tests."""
    img = Image.new("RGB", (4, 4))
    pixels = [
        (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255),
        (128, 128, 128), (0, 0, 0), (255, 255, 0), (0, 255, 255),
        (255, 0, 255), (64, 64, 64), (192, 192, 192), (128, 0, 0),
        (0, 128, 0), (0, 0, 128), (128, 128, 0), (0, 128, 128),
    ]
    for i, px in enumerate(pixels):
        img.putpixel((i % 4, i // 4), px)
    path = tmp_path / "test_input.bmp"
    img.save(str(path), "BMP")
    return str(path)


@pytest.fixture
def sample_large_image(tmp_path):
    """Create a 200x100 image for resize tests."""
    img = Image.new("RGB", (200, 100), color=(100, 150, 200))
    path = tmp_path / "test_large.bmp"
    img.save(str(path), "BMP")
    return str(path)
