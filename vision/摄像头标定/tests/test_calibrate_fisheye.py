from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from calibrate_fisheye import (  # noqa: E402
    detect_chessboard_corners,
    iter_images,
    read_grayscale_image,
    run_calibration,
)
from generate_firmware_headers import c_float  # noqa: E402


class CalibrationRegressionTests(unittest.TestCase):
    def test_c_float_always_emits_valid_float_literal(self) -> None:
        self.assertEqual("0.0f", c_float(0.0))
        self.assertEqual("2.0f", c_float(2))
        self.assertEqual("-0.125f", c_float(-0.125))

    def test_iter_images_recurses_into_capture_subdirectories(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            nested = root / "uart_capture" / "png"
            nested.mkdir(parents=True)
            cv2.imwrite(str(nested / "frame.png"), np.zeros((10, 10), np.uint8))
            (root / "ignored.raw").write_bytes(b"raw")

            images = list(iter_images(root))

            self.assertEqual([nested / "frame.png"], images)

    def test_low_resolution_11_by_8_corner_detection(self) -> None:
        square_px = 8
        square_cols = 12
        square_rows = 9
        board = np.zeros((square_rows * square_px, square_cols * square_px), np.uint8)
        for row in range(square_rows):
            for col in range(square_cols):
                if (row + col) % 2:
                    board[
                        row * square_px : (row + 1) * square_px,
                        col * square_px : (col + 1) * square_px,
                    ] = 255
        image = np.full((120, 188), 127, np.uint8)
        y0 = (image.shape[0] - board.shape[0]) // 2
        x0 = (image.shape[1] - board.shape[1]) // 2
        image[y0 : y0 + board.shape[0], x0 : x0 + board.shape[1]] = board

        corners, detector = detect_chessboard_corners(image, (11, 8))

        self.assertIsNotNone(corners)
        self.assertEqual((1, 88, 2), corners.shape)
        self.assertIn(detector, {"findChessboardCornersSB", "findChessboardCorners"})

    def test_read_grayscale_image_supports_chinese_paths(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "中文目录" / "标定图.png"
            path.parent.mkdir()
            source = np.arange(100, dtype=np.uint8).reshape(10, 10)
            ok, encoded = cv2.imencode(".png", source)
            self.assertTrue(ok)
            path.write_bytes(encoded.tobytes())

            loaded = read_grayscale_image(path)

            self.assertIsNotNone(loaded)
            np.testing.assert_array_equal(source, loaded)

    def test_current_188_by_120_capture_set_calibrates(self) -> None:
        frames_dir = PROJECT_ROOT / "frames" / "uart_capture"
        self.assertEqual(19, len(list(iter_images(frames_dir))))

        with tempfile.TemporaryDirectory() as tmp:
            result = run_calibration(
                frames_dir=frames_dir,
                output_dir=tmp,
                pattern_cols=11,
                pattern_rows=8,
                square_size_cm=2.0,
            )

        self.assertGreaterEqual(result["used_count"], 10)
        self.assertLess(result["rms"], 1.0)
        self.assertEqual(188, result["image_width"])
        self.assertEqual(120, result["image_height"])
        self.assertEqual(0.0, result["valid_radius_px"])


if __name__ == "__main__":
    unittest.main()
