"""
test_ocr.py

Standalone test script — no ROS needed.

Run this to verify that EasyOCR can read a digital clock from a photo
and that the validation logic (same logic as in read_time_service.py)
produces the correct result.

Usage:
    python3 test_ocr.py                      # uses test_clock.jpg in the same folder
    python3 test_ocr.py my_other_photo.jpg   # use a different image
"""

import re
import sys
import os

import cv2
import easyocr


# --------------------------------------------------------------------------- #
# Configuration                                                                 #
# --------------------------------------------------------------------------- #

# Default image: test_clock.jpg sitting next to this script.
DEFAULT_IMAGE = os.path.join(os.path.dirname(__file__), 'test_clock.jpg')

# Same confidence threshold as the ROS node default.
CONFIDENCE_THRESHOLD = 0.7


# --------------------------------------------------------------------------- #
# Validation logic (copy of _validate_ocr from read_time_service.py)          #
# --------------------------------------------------------------------------- #

def validate_ocr(results, threshold=CONFIDENCE_THRESHOLD):
    """
    Check whether OCR results contain exactly one valid NN:NN time string.

    Accepts a detection only if:
      - Text matches exactly ^\d{2}:\d{2}$  (e.g. '14:32')
      - Confidence >= threshold

    Returns (True, [d0,d1,d2,d3]) on success, (False, []) otherwise.
    """
    pattern = re.compile(r'^\d{2}:\d{2}$')
    accepted = []

    for _bbox, text, confidence in results:
        text = text.strip()
        format_ok = bool(pattern.match(text))
        conf_ok   = confidence >= threshold
        if format_ok and conf_ok:
            accepted.append(text)

    if len(accepted) == 1:
        digits = [int(c) for c in accepted[0] if c.isdigit()]
        return True, digits

    return False, []


# --------------------------------------------------------------------------- #
# Main                                                                          #
# --------------------------------------------------------------------------- #

def main():
    # Allow passing a different image as a command-line argument.
    image_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IMAGE

    if not os.path.isfile(image_path):
        print(f'ERROR: image not found: {image_path}')
        sys.exit(1)

    print(f'Image       : {image_path}')
    print(f'Threshold   : {CONFIDENCE_THRESHOLD}')
    print()

    # Load the image with OpenCV so we can display its size.
    img = cv2.imread(image_path)
    if img is None:
        print('ERROR: OpenCV could not read the image.')
        sys.exit(1)
    h, w = img.shape[:2]
    print(f'Image size  : {w} x {h} px')
    print()

    # Load EasyOCR (model is cached after first download).
    print('Loading EasyOCR model...')
    reader = easyocr.Reader(['en'], gpu=False, verbose=False)
    print('Model loaded.')
    print()

    # Run OCR restricted to digits and colons (same as in the ROS node).
    print('Running OCR...')
    results = reader.readtext(image_path, allowlist='0123456789:')
    print(f'Found {len(results)} detection(s).')
    print()

    # Print every detection with its confidence and format check.
    pattern = re.compile(r'^\d{2}:\d{2}$')
    print('--- Raw OCR detections ---')
    if not results:
        print('  (none)')
    for i, (_bbox, text, confidence) in enumerate(results):
        text       = text.strip()
        format_ok  = bool(pattern.match(text))
        conf_ok    = confidence >= CONFIDENCE_THRESHOLD
        verdict    = 'ACCEPTED' if (format_ok and conf_ok) else 'rejected'
        print(f'  [{i}] "{text}"  confidence={confidence:.3f}  '
              f'format={format_ok}  conf_ok={conf_ok}  → {verdict}')
    print()

    # Run the same validation as the ROS node.
    found, digits = validate_ocr(results)

    print('--- Validation result ---')
    if found:
        time_str = ''.join(str(d) for d in digits)
        print(f'  SUCCESS: found={found}  digits={digits}  → {time_str[:2]}:{time_str[2:]}')
    else:
        print(f'  FAILED : found={found}  digits=[]')
        if len(results) == 0:
            print('  Reason : no text detected at all.')
        else:
            accepted = [t for _, t, c in results
                        if pattern.match(t.strip()) and c >= CONFIDENCE_THRESHOLD]
            if len(accepted) == 0:
                print('  Reason : detections found but none matched NN:NN with '
                      f'confidence >= {CONFIDENCE_THRESHOLD}.')
                print('  Tip    : try lowering CONFIDENCE_THRESHOLD in this script, '
                      'or check if the clock is clearly visible.')
            elif len(accepted) > 1:
                print(f'  Reason : {len(accepted)} matches found (ambiguous): {accepted}')


if __name__ == '__main__':
    main()
