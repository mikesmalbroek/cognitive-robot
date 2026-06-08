"""
test_detect_abacus_service.py

Unit tests for detect_abacus_service.py.

These tests do NOT need a real robot, camera, or internet connection:
  - ROS2 is initialised once for the whole module via a pytest fixture.
  - inference_sdk is inserted into sys.modules as a MagicMock before the
    service module is imported, so no real package needs to be installed.
  - Camera frames are injected directly into node.latest_frame.

Run with:
    python3 -m pytest test/test_detect_abacus_service.py -v
"""

import os
from unittest.mock import MagicMock

# inference_sdk is mocked in conftest.py before this file is imported.

import numpy as np
import pytest
import rclpy

from cognitive_robot.detect_abacus_service import DetectAbacusService
from cognitive_robot_interfaces.srv import DetectAbacus


# --------------------------------------------------------------------------- #
# Fixtures                                                                      #
# --------------------------------------------------------------------------- #

@pytest.fixture(scope='module')
def ros_context():
    """Initialise and shut down the ROS2 context once for the whole module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    """
    Create a DetectAbacusService node.

    inference_sdk is already mocked at module level, so no real HTTP
    connection is attempted during node creation.
    """
    n = DetectAbacusService()
    yield n
    n.destroy_node()


@pytest.fixture
def fake_frame():
    """A small black BGR image used as a stand-in for a real camera frame."""
    return np.zeros((480, 640, 3), dtype=np.uint8)


@pytest.fixture
def response():
    """An empty DetectAbacus response to pass into _handle_detect_abacus."""
    return DetectAbacus.Response()


# --------------------------------------------------------------------------- #
# Tests for _extract_best_detection                                            #
# --------------------------------------------------------------------------- #

class TestExtractBestDetection:
    """Tests for the _extract_best_detection base function."""

    def test_empty_predictions_returns_zeros(self, node):
        """No predictions → all return values are zero / 0.0."""
        confidence, x, y = node._extract_best_detection([])
        assert confidence == 0.0
        assert x == 0
        assert y == 0

    def test_single_prediction_above_threshold(self, node):
        """One prediction above the threshold → its values are returned."""
        predictions = [{'x': 100.0, 'y': 200.0, 'confidence': 0.9}]
        confidence, x, y = node._extract_best_detection(predictions)
        assert confidence == pytest.approx(0.9)
        assert x == 100
        assert y == 200

    def test_single_prediction_below_threshold(self, node):
        """One prediction below the threshold → treated as no detection."""
        # Default threshold is 0.5; 0.2 is well below it.
        predictions = [{'x': 100.0, 'y': 200.0, 'confidence': 0.2}]
        confidence, x, y = node._extract_best_detection(predictions)
        assert confidence == 0.0
        assert x == 0
        assert y == 0

    def test_picks_highest_confidence_from_multiple(self, node):
        """Multiple predictions → the one with the highest confidence wins."""
        predictions = [
            {'x': 10.0, 'y': 10.0, 'confidence': 0.6},
            {'x': 50.0, 'y': 80.0, 'confidence': 0.95},
            {'x': 30.0, 'y': 40.0, 'confidence': 0.75},
        ]
        confidence, x, y = node._extract_best_detection(predictions)
        assert confidence == pytest.approx(0.95)
        assert x == 50
        assert y == 80

    def test_all_below_threshold_returns_zeros(self, node):
        """Multiple predictions all below the threshold → no detection."""
        predictions = [
            {'x': 10.0, 'y': 10.0, 'confidence': 0.1},
            {'x': 50.0, 'y': 80.0, 'confidence': 0.3},
        ]
        confidence, x, y = node._extract_best_detection(predictions)
        assert confidence == 0.0
        assert x == 0
        assert y == 0


# --------------------------------------------------------------------------- #
# Tests for _capture_frame                                                     #
# --------------------------------------------------------------------------- #

class TestCaptureFrame:
    """Tests for the _capture_frame base function."""

    def test_returns_none_when_no_frame_received(self, node):
        """Before any camera message arrives, _capture_frame returns None."""
        node.latest_frame = None
        assert node._capture_frame() is None

    def test_returns_copy_not_original_reference(self, node, fake_frame):
        """_capture_frame must return a copy so the caller cannot corrupt the stored frame."""
        node.latest_frame = fake_frame
        result = node._capture_frame()

        # Different object in memory — modifying result must not affect the stored frame.
        assert result is not fake_frame
        # But the pixel data must be identical.
        assert np.array_equal(result, fake_frame)


# --------------------------------------------------------------------------- #
# Tests for _save_temp_image                                                   #
# --------------------------------------------------------------------------- #

class TestSaveTempImage:
    """Tests for the _save_temp_image base function."""

    def test_creates_file_on_disk(self, node, fake_frame):
        """After calling _save_temp_image, the JPEG file must exist on disk."""
        path = node._save_temp_image(fake_frame)
        assert os.path.isfile(path)

    def test_returns_non_empty_string_path(self, node, fake_frame):
        """The return value must be a non-empty file path string."""
        path = node._save_temp_image(fake_frame)
        assert isinstance(path, str)
        assert len(path) > 0


# --------------------------------------------------------------------------- #
# Tests for _run_inference                                                     #
# --------------------------------------------------------------------------- #

class TestRunInference:
    """Tests for the _run_inference base function."""

    def test_returns_predictions_on_success(self, node, fake_frame, tmp_path):
        """When the API responds normally, the prediction list is returned."""
        img_path = str(tmp_path / 'test.jpg')
        node._save_temp_image(fake_frame)

        fake_predictions = [
            {'x': 320.0, 'y': 240.0, 'confidence': 0.88, 'class': 'abacus'}
        ]
        node._inference_client.infer = MagicMock(
            return_value={'predictions': fake_predictions}
        )

        result = node._run_inference(img_path)
        assert result == fake_predictions

    def test_returns_empty_list_on_api_exception(self, node, tmp_path):
        """When the API raises an exception, an empty list is returned without crashing."""
        node._inference_client.infer = MagicMock(side_effect=RuntimeError('timeout'))
        result = node._run_inference(str(tmp_path / 'test.jpg'))
        assert result == []


# --------------------------------------------------------------------------- #
# Tests for _handle_detect_abacus (integration)                               #
# --------------------------------------------------------------------------- #

class TestHandleDetectAbacus:
    """Integration tests for the _handle_detect_abacus service callback."""

    def test_no_frame_returns_zero_confidence(self, node, response):
        """If no camera frame is available, confidence must be 0.0 and coords 0."""
        node.latest_frame = None
        node._handle_detect_abacus(DetectAbacus.Request(), response)
        assert response.confidence == 0.0
        assert response.x == 0
        assert response.y == 0

    def test_detection_fills_response_correctly(self, node, fake_frame, response):
        """When inference finds a good detection, the response fields are filled correctly."""
        node.latest_frame = fake_frame
        node._inference_client.infer = MagicMock(return_value={
            'predictions': [
                {'x': 150.0, 'y': 250.0, 'confidence': 0.85, 'class': 'abacus'}
            ]
        })
        node._handle_detect_abacus(DetectAbacus.Request(), response)
        assert response.confidence == pytest.approx(0.85)
        assert response.x == 150
        assert response.y == 250

    def test_no_predictions_returns_zero_confidence(self, node, fake_frame, response):
        """When inference returns no predictions, confidence must be 0.0."""
        node.latest_frame = fake_frame
        node._inference_client.infer = MagicMock(return_value={'predictions': []})
        node._handle_detect_abacus(DetectAbacus.Request(), response)
        assert response.confidence == 0.0
        assert response.x == 0
        assert response.y == 0

    def test_api_failure_returns_zero_confidence(self, node, fake_frame, response):
        """When the API call fails entirely, confidence must be 0.0 (no crash)."""
        node.latest_frame = fake_frame
        node._inference_client.infer = MagicMock(side_effect=RuntimeError('connection error'))
        node._handle_detect_abacus(DetectAbacus.Request(), response)
        assert response.confidence == 0.0
        assert response.x == 0
        assert response.y == 0
