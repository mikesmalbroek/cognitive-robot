"""
test_roboflow.py

Standalone test script — geen ROS, geen robot nodig.

Stuurt een lokale foto naar de Roboflow API en print wat het ziet.
Gebruik dit om te controleren of het Roboflow model werkt
voordat je het op de echte robot test.

Gebruik
-------
    python3 test_roboflow.py /pad/naar/foto.jpg

Voorbeeld
---------
    python3 test_roboflow.py /tmp/detect_abacus_temp.jpg
    python3 test_roboflow.py ~/photos/photo_0000.jpg
"""

import sys

from inference_sdk import InferenceHTTPClient

API_URL   = 'https://serverless.roboflow.com'
API_KEY   = '8U4Olre0d5v9lWGCeHHT'
MODEL_ID  = 'abacus_recognition_v1/2'


def main():
    if len(sys.argv) < 2:
        print('Gebruik: python3 test_roboflow.py /pad/naar/foto.jpg')
        sys.exit(1)

    image_path = sys.argv[1]
    print(f'Foto: {image_path}')
    print(f'Model: {MODEL_ID}')
    print('Bezig met versturen naar Roboflow...\n')

    client = InferenceHTTPClient(api_url=API_URL, api_key=API_KEY)
    client.select_api_v0()  # skip localhost:9001 check, go straight to cloud
    result = client.infer(image_path, model_id=MODEL_ID)
    predictions = result.get('predictions', [])

    if not predictions:
        print('Geen abacus gevonden.')
        return

    print(f'{len(predictions)} detectie(s) gevonden:\n')
    for i, p in enumerate(predictions):
        print(f'  [{i+1}] confidence={p["confidence"]:.2f}  '
              f'x={int(p["x"])}  y={int(p["y"])}  '
              f'breedte={int(p["width"])}  hoogte={int(p["height"])}')


if __name__ == '__main__':
    main()
