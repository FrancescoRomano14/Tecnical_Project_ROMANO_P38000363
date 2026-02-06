#!/usr/bin/env python3
"""Rotate the arena map image by 45 degrees"""

from PIL import Image
import sys

# Open the original image
img = Image.open('arena_map.pgm')

# Rotate by -45 degrees (clockwise) with white background
rotated = img.rotate(45, expand=True, fillcolor=255)

# Save the rotated image
rotated.save('arena_map_rotated.pgm')

print(f"Original size: {img.size}")
print(f"Rotated size: {rotated.size}")
print("Saved as: arena_map_rotated.pgm")
