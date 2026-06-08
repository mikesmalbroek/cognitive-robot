""" 
This python file will detect stations in images and output the results in a JSON file. 

It uses OpenCV to process the images and detect the stations based on color ranges in the HSV color space. 
The results include the station type (A or B) and the bounding box coordinates for each detected station. 
The code is designed to be flexible, allowing users to set custom color ranges if needed.
"""

import cv2
import numpy as np
from pathlib import Path
import json


class StationDetector:
    
    def __init__(self, min_area=50):
        self.min_area = min_area
        self.ranges = {
            'A': {'h': (0, 15), 's': (100, 255), 'v': (50, 255)},
            'A_alt': {'h': (170, 180), 's': (100, 255), 'v': (50, 255)},
            'B': {'h': (100, 130), 's': (100, 255), 'v': (50, 255)}
        }
    
    def set_range(self, station, h, s, v):
        self.ranges[station] = {'h': h, 's': s, 'v': v}
    
    def detect(self, image_path):
        img = cv2.imread(image_path)
        if img is None:
            return None
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Check for red (A)
        mask_a = self._make_mask(hsv, self.ranges['A'])
        mask_a_alt = self._make_mask(hsv, self.ranges['A_alt'])
        mask_a = cv2.bitwise_or(mask_a, mask_a_alt)
        
        result = self._get_bbox(mask_a)
        if result:
            return ('A', result)
        
        # Check for blue (B)
        mask_b = self._make_mask(hsv, self.ranges['B'])
        result = self._get_bbox(mask_b)
        if result:
            return ('B', result)
        
        return None
    
    def _make_mask(self, hsv, r):
        lower = np.array([r['h'][0], r['s'][0], r['v'][0]])
        upper = np.array([r['h'][1], r['s'][1], r['v'][1]])
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask
    
    def _get_bbox(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.min_area:
            return None
        
        x, y, w, h = cv2.boundingRect(largest)
        return (x, y, w, h)
    
    def process_folder(self, folder):
        folder_path = Path(folder)
        results = {}
        
        for img_file in sorted(folder_path.glob('*.jpg')):
            result = self.detect(str(img_file))
            results[img_file.name] = result
            print(f"{img_file.name}: {result}")
        
        for img_file in sorted(folder_path.glob('*.png')):
            result = self.detect(str(img_file))
            results[img_file.name] = result
            print(f"{img_file.name}: {result}")
        
        return results
    
    def save_json(self, results, output_file):
        data = {}
        for name, result in results.items():
            if result:
                station, (x, y, w, h) = result
                data[name] = {'station': station, 'x': x, 'y': y, 'w': w, 'h': h}
            else:
                data[name] = {'station': None}
        
        with open(output_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Saved to {output_file}")
    
    def visualize_folder(self, folder, output_folder='./annotated'):
        folder_path = Path(folder)
        output_path = Path(output_folder)
        output_path.mkdir(exist_ok=True)
        
        for img_file in sorted(folder_path.glob('*.jpg')) + sorted(folder_path.glob('*.png')):
            img = cv2.imread(str(img_file))
            result = self.detect(str(img_file))
            
            if result:
                station, (x, y, w, h) = result
                color = (0, 0, 255) if station == 'A' else (255, 0, 0)
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, f'Station {station}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            output_file = output_path / img_file.name
            cv2.imwrite(str(output_file), img)
            print(f"Saved: {output_file}")


if __name__ == '__main__':
    script_dir = Path(__file__).parent

    detector = StationDetector(min_area=50)

    # Set custom ranges if needed
    # detector.set_range('A', h=(0, 15), s=(100, 255), v=(50, 255))
    # detector.set_range('B', h=(100, 130), s=(100, 255), v=(50, 255))

    # Process folder and visualize
    detector.visualize_folder(script_dir / 'images', script_dir / 'results')
    results = detector.process_folder(script_dir / 'images')
    detector.save_json(results, script_dir / 'results.json')