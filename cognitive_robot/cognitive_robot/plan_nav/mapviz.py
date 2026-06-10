#!/usr/bin/env python3

import os
import math
import yaml
import cv2
import numpy as np


_HERE = os.path.dirname(os.path.abspath(__file__))

MAP_YAML = os.path.join(_HERE, "auto_map.yaml")

STATION_FILES = [
    {
        "yaml": os.path.join(_HERE, "station_a_location.yaml"),
        "label": "Station A",
        "color": (0, 0, 255),      # red in BGR
    },
    {
        "yaml": os.path.join(_HERE, "station_b_location.yaml"),
        "label": "Station B",
        "color": (255, 0, 0),      # blue in BGR
    },
    {
        "yaml": os.path.join(_HERE, "abacus_location.yaml"),
        "label": "Abacus",
        "color": (0, 255, 0),      # green in BGR
    },
]

OUTPUT_IMAGE = os.path.join(_HERE, "trial_map_with_stations_and_destinations.png")

# Increase this to make the final displayed/saved map larger
DISPLAY_SCALE = 6

# Station marker size before scaling
STATION_DOT_RADIUS = 1
STATION_OUTLINE_RADIUS = 1

# Destination marker size AFTER scaling
DESTINATION_DOT_RADIUS = 4
DESTINATION_OUTLINE_RADIUS = 6

# Arrow size AFTER scaling
ARROW_LENGTH_PIXELS = 30
ARROW_THICKNESS = 2
ARROW_TIP_LENGTH = 0.35

# Labels
DRAW_LABELS = False
LABEL_SCALE = 0.35
LABEL_THICKNESS = 1


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def map_to_pixel(x, y, origin_x, origin_y, resolution, image_height):
    pixel_x = int((x - origin_x) / resolution)

    # ROS map y-axis points upward, image y-axis points downward
    pixel_y = image_height - int((y - origin_y) / resolution)

    return pixel_x, pixel_y


def draw_station(
    image,
    station_name,
    station_x,
    station_y,
    pixel_x,
    pixel_y,
    color,
    width,
    height
):
    if 0 <= pixel_x < width and 0 <= pixel_y < height:
        # Filled station dot
        cv2.circle(
            image,
            (pixel_x, pixel_y),
            radius=STATION_DOT_RADIUS,
            color=color,
            thickness=-1
        )

        # Small black outline
        if STATION_OUTLINE_RADIUS > 0:
            cv2.circle(
                image,
                (pixel_x, pixel_y),
                radius=STATION_OUTLINE_RADIUS,
                color=(0, 0, 0),
                thickness=1
            )

        if DRAW_LABELS:
            cv2.putText(
                image,
                station_name,
                (pixel_x + 8, pixel_y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                LABEL_SCALE,
                color,
                LABEL_THICKNESS
            )

        print(f"{station_name} station marker drawn successfully.")

    else:
        print(f"WARNING: {station_name} station marker is outside the map image boundaries.")
        print(f"  Map position   : x={station_x:+.3f}, y={station_y:+.3f}")
        print(f"  Pixel position : px={pixel_x}, py={pixel_y}")


def draw_destination_after_scaling(
    image,
    station_name,
    destination_x,
    destination_y,
    destination_yaw,
    pixel_x,
    pixel_y,
    original_width,
    original_height
):
    """
    Draw destination point and robot yaw arrow on the already-scaled image.

    ROS/map yaw:
        +x is right in map
        +y is up in map

    Image pixel frame:
        +x is right
        +y is down

    Therefore:
        arrow_end_x = start_x + cos(yaw)
        arrow_end_y = start_y - sin(yaw)
    """

    scaled_width = original_width * DISPLAY_SCALE
    scaled_height = original_height * DISPLAY_SCALE

    scaled_x = int(pixel_x * DISPLAY_SCALE)
    scaled_y = int(pixel_y * DISPLAY_SCALE)

    if not (0 <= scaled_x < scaled_width and 0 <= scaled_y < scaled_height):
        print(f"WARNING: {station_name} destination is outside the map image boundaries.")
        print(f"  Destination map position   : x={destination_x:+.3f}, y={destination_y:+.3f}")
        print(f"  Destination pixel position : px={pixel_x}, py={pixel_y}")
        return

    green = (0, 255, 0)

    # Green destination dot
    cv2.circle(
        image,
        (scaled_x, scaled_y),
        radius=DESTINATION_DOT_RADIUS,
        color=green,
        thickness=-1
    )

    # Black outline
    if DESTINATION_OUTLINE_RADIUS > 0:
        cv2.circle(
            image,
            (scaled_x, scaled_y),
            radius=DESTINATION_OUTLINE_RADIUS,
            color=(0, 0, 0),
            thickness=1
        )

    # Arrow showing robot alignment / yaw
    arrow_end_x = int(scaled_x + ARROW_LENGTH_PIXELS * math.cos(destination_yaw))
    arrow_end_y = int(scaled_y - ARROW_LENGTH_PIXELS * math.sin(destination_yaw))

    cv2.arrowedLine(
        image,
        (scaled_x, scaled_y),
        (arrow_end_x, arrow_end_y),
        green,
        thickness=ARROW_THICKNESS,
        tipLength=ARROW_TIP_LENGTH
    )

    if DRAW_LABELS:
        cv2.putText(
            image,
            f"{station_name} goal",
            (scaled_x + 10, scaled_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            green,
            1
        )

    print(f"{station_name} destination and yaw arrow drawn successfully.")


def main():
    # -------------------------------------------------------------------------
    # Load map metadata
    # -------------------------------------------------------------------------
    map_data = load_yaml(MAP_YAML)

    map_image_name = map_data["image"]
    resolution = float(map_data["resolution"])
    origin_x = float(map_data["origin"][0])
    origin_y = float(map_data["origin"][1])

    map_yaml_dir = os.path.dirname(os.path.abspath(MAP_YAML))
    map_image_path = os.path.join(map_yaml_dir, map_image_name)

    # -------------------------------------------------------------------------
    # Load map image
    # -------------------------------------------------------------------------
    map_img = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)

    if map_img is None:
        raise FileNotFoundError(
            f"Could not load map image: {map_image_path}\n"
            f"Make sure {map_image_name} is in the same folder as {MAP_YAML}."
        )

    height, width = map_img.shape

    # Convert grayscale map to color image so we can draw colored dots
    map_color = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)

    print("=" * 60)
    print("Map information")
    print("=" * 60)
    print(f"Map image        : {map_image_path}")
    print(f"Map origin       : x={origin_x:+.3f} m, y={origin_y:+.3f} m")
    print(f"Map resolution   : {resolution:.3f} m/pixel")
    print(f"Original size    : width={width}, height={height}")
    print(f"Display scale    : {DISPLAY_SCALE}x")
    print(f"Final size       : width={width * DISPLAY_SCALE}, height={height * DISPLAY_SCALE}")
    print("=" * 60)

    destination_draw_list = []

    # -------------------------------------------------------------------------
    # Load and draw each station marker on the original map image
    # -------------------------------------------------------------------------
    for station_file in STATION_FILES:
        station_yaml = station_file["yaml"]
        fallback_label = station_file["label"]
        color = station_file["color"]

        if not os.path.exists(station_yaml):
            print()
            print(f"WARNING: {station_yaml} not found. Skipping {fallback_label}.")
            continue

        station_data = load_yaml(station_yaml)

        station_name = station_data.get("station_name", fallback_label)

        # ---------------------------------------------------------------------
        # Draw actual station marker location
        # ---------------------------------------------------------------------
        station_x = float(station_data["map_pose"]["x"])
        station_y = float(station_data["map_pose"]["y"])

        station_pixel_x, station_pixel_y = map_to_pixel(
            station_x,
            station_y,
            origin_x,
            origin_y,
            resolution,
            height
        )

        print()
        print("-" * 60)
        print(f"Station file      : {station_yaml}")
        print(f"Station name      : {station_name}")
        print(f"Station position  : x={station_x:+.3f} m, y={station_y:+.3f} m")
        print(f"Station pixel     : px={station_pixel_x}, py={station_pixel_y}")

        draw_station(
            image=map_color,
            station_name=station_name,
            station_x=station_x,
            station_y=station_y,
            pixel_x=station_pixel_x,
            pixel_y=station_pixel_y,
            color=color,
            width=width,
            height=height
        )

        # ---------------------------------------------------------------------
        # Read destination pose from same station YAML
        # ---------------------------------------------------------------------
        if "destination_pose" not in station_data:
            print(f"WARNING: {station_yaml} has no destination_pose block.")
            print("Press b again with the updated mapper code to regenerate this file.")
            print("-" * 60)
            continue

        destination_x = float(station_data["destination_pose"]["x"])
        destination_y = float(station_data["destination_pose"]["y"])
        destination_yaw = float(station_data["destination_pose"]["yaw_rad"])

        destination_pixel_x, destination_pixel_y = map_to_pixel(
            destination_x,
            destination_y,
            origin_x,
            origin_y,
            resolution,
            height
        )

        print(f"Destination pos   : x={destination_x:+.3f} m, y={destination_y:+.3f} m")
        print(f"Destination yaw   : {math.degrees(destination_yaw):+.1f} deg")
        print(f"Destination pixel : px={destination_pixel_x}, py={destination_pixel_y}")
        print("-" * 60)

        destination_draw_list.append({
            "station_name": station_name,
            "x": destination_x,
            "y": destination_y,
            "yaw": destination_yaw,
            "pixel_x": destination_pixel_x,
            "pixel_y": destination_pixel_y,
        })

    # -------------------------------------------------------------------------
    # Scale up final image
    # -------------------------------------------------------------------------
    large_map = cv2.resize(
        map_color,
        None,
        fx=DISPLAY_SCALE,
        fy=DISPLAY_SCALE,
        interpolation=cv2.INTER_NEAREST
    )

    # -------------------------------------------------------------------------
    # Draw destination points and yaw arrows AFTER scaling
    # This keeps the green goal dots/arrows visually small and clean.
    # -------------------------------------------------------------------------
    for dest in destination_draw_list:
        draw_destination_after_scaling(
            image=large_map,
            station_name=dest["station_name"],
            destination_x=dest["x"],
            destination_y=dest["y"],
            destination_yaw=dest["yaw"],
            pixel_x=dest["pixel_x"],
            pixel_y=dest["pixel_y"],
            original_width=width,
            original_height=height
        )

    # -------------------------------------------------------------------------
    # Save and display result
    # -------------------------------------------------------------------------
    cv2.imwrite(OUTPUT_IMAGE, large_map)

    print()
    print("=" * 60)
    print(f"Saved enlarged overlay image as: {OUTPUT_IMAGE}")
    print("=" * 60)

    cv2.imshow("SLAM Map with Stations and Robot Destination Goals", large_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()