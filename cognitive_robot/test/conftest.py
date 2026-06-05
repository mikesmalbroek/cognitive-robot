"""
conftest.py

Pytest configuration loaded automatically before any test module.

We mock inference_sdk here (rather than inside a test file) because pytest's
launch_testing plugin imports test files early — before module-level code in
those files can run. By placing the mock in conftest.py we guarantee it is
active before detect_abacus_service is imported anywhere.
"""

import sys
from unittest.mock import MagicMock

sys.modules['inference_sdk'] = MagicMock()
