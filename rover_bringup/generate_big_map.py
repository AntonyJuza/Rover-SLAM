import os
import numpy as np
from PIL import Image, ImageDraw

def create_map():
    # 26m x 26m area at 0.05m/pixel resolution -> 520 x 520 pixels
    resolution = 0.05
    width_m = 26.0
    height_m = 26.0
    width_px = int(width_m / resolution)   # 520
    height_px = int(height_m / resolution) # 520

    origin_x = -13.0
    origin_y = -13.0

    # Create image initialized to white (free space = 254)
    img = Image.new('L', (width_px, height_px), 254)
    draw = ImageDraw.Draw(img)

    def world_to_px(x, y):
        # Convert world coordinates (meters) to pixel coordinates (X right, Y down)
        px = int((x - origin_x) / resolution)
        py = int((height_m - (y - origin_y)) / resolution)
        return px, py

    def draw_box(center_x, center_y, size_x, size_y, rot_rad=0):
        # Calculate corners of box
        hw = size_x / 2.0
        hh = size_y / 2.0
        corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        rotated = []
        cos_r = np.cos(rot_rad)
        sin_r = np.sin(rot_rad)
        for cx, cy in corners:
            rx = center_x + cx * cos_r - cy * sin_r
            ry = center_y + cx * sin_r + cy * cos_r
            rotated.append(world_to_px(rx, ry))
        draw.polygon(rotated, fill=0)

    def draw_cylinder(center_x, center_y, radius):
        px_c, py_c = world_to_px(center_x, center_y)
        r_px = int(radius / resolution)
        draw.ellipse([px_c - r_px, py_c - r_px, px_c + r_px, py_c + r_px], fill=0)

    # Outer Walls (25m x 25m)
    draw_box(0, 12.5, 25.2, 0.2)
    draw_box(0, -12.5, 25.2, 0.2)
    draw_box(12.5, 0, 0.2, 25.2)
    draw_box(-12.5, 0, 0.2, 25.2)

    # Interior Partitions
    draw_box(5, 5, 8.0, 0.3)
    draw_box(-5, -5, 8.0, 0.3)
    draw_box(-6, 4, 6.0, 0.3, rot_rad=1.57)

    # Static Pillars
    draw_cylinder(7, -7, 0.8)
    draw_box(-8, 8, 1.5, 1.5)
    draw_cylinder(9, 9, 0.6)
    draw_box(-9, -9, 1.2, 1.2)
    draw_cylinder(3, -8, 0.5)
    draw_box(-3, 8, 1.0, 1.0)

    # Save PGM image
    pgm_path = '/home/juza/rover-rpi/my_map.pgm'
    img.save(pgm_path)

    # Save YAML configuration
    yaml_content = f"""image: my_map.pgm
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    yaml_path = '/home/juza/rover-rpi/my_map.yaml'
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)

    print(f"Generated map: {pgm_path} and {yaml_path}")

if __name__ == '__main__':
    create_map()
