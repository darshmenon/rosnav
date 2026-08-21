#!/usr/bin/env python3
"""Procedural texture generator — small, git-friendly PNGs for house.world /
hospital.world floors & walls so RTAB-Map / visual SLAM has real texture to
track (flat ambient/diffuse colors give feature detectors nothing to lock
onto). Patterns are baked at full resolution so a single non-tiled UV map
still reads as a tiled floor. Run: python3 gen_world_textures.py"""
import os
import random
from PIL import Image, ImageDraw, ImageFilter

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(SCRIPT_DIR, '..', 'models', 'textures')
os.makedirs(OUT, exist_ok=True)
random.seed(42)
N = 512


def noise_layer(size, amount):
    return Image.effect_noise((size, size), amount).convert("L")


def save(img, name):
    path = os.path.join(OUT, name)
    img.convert("RGB").save(path, optimize=True)
    print(name, os.path.getsize(path) // 1024, "KB")


def wood_plank():
    img = Image.new("RGB", (N, N))
    draw = ImageDraw.Draw(img)
    plank_h = N // 8
    for row in range(8):
        base = (150 + random.randint(-15, 15), 105 + random.randint(-12, 12), 65 + random.randint(-10, 10))
        y0, y1 = row * plank_h, (row + 1) * plank_h
        draw.rectangle([0, y0, N, y1], fill=base)
        for x in range(0, N, 3):
            shade = random.randint(-8, 8)
            c = tuple(max(0, min(255, v + shade)) for v in base)
            draw.line([(x, y0), (x, y1)], fill=c)
        draw.line([(0, y1 - 1), (N, y1 - 1)], fill=(60, 40, 25), width=2)
    for _ in range(40):
        x0 = random.randint(0, N)
        row = random.randint(0, 7)
        draw.line([(x0, row * plank_h), (x0, (row + 1) * plank_h)], fill=(70, 45, 25), width=1)
    img = img.filter(ImageFilter.GaussianBlur(0.4))
    save(img, "wood_plank.png")


def tile_grid(name, base_color, grout=(120, 120, 120), tiles=8, speck=10):
    img = Image.new("RGB", (N, N), base_color)
    n = noise_layer(N, speck)
    img = Image.composite(Image.new("RGB", (N, N), tuple(min(255, c + 15) for c in base_color)),
                           img, n.point(lambda p: 255 if p > 150 else 0))
    draw = ImageDraw.Draw(img)
    step = N // tiles
    for i in range(tiles + 1):
        draw.line([(i * step, 0), (i * step, N)], fill=grout, width=2)
        draw.line([(0, i * step), (N, i * step)], fill=grout, width=2)
    save(img, name)


def carpet(name, base_color):
    img = Image.new("RGB", (N, N), base_color)
    n = noise_layer(N, 25)
    speckled = Image.composite(
        Image.new("RGB", (N, N), tuple(max(0, c - 30) for c in base_color)),
        Image.composite(Image.new("RGB", (N, N), tuple(min(255, c + 25) for c in base_color)),
                         img, n.point(lambda p: 255 if p > 170 else 0)),
        n.point(lambda p: 255 if p < 80 else 0))
    speckled = speckled.filter(ImageFilter.GaussianBlur(0.6))
    save(speckled, name)


def concrete():
    base = (150, 148, 143)
    img = Image.new("RGB", (N, N), base)
    n = noise_layer(N, 30)
    img = Image.composite(Image.new("RGB", (N, N), tuple(max(0, c - 25) for c in base)),
                           img, n.point(lambda p: 255 if p > 180 else 0))
    draw = ImageDraw.Draw(img)
    for _ in range(6):
        x0, y0 = random.randint(0, N), random.randint(0, N)
        x1, y1 = x0 + random.randint(-120, 120), y0 + random.randint(-120, 120)
        draw.line([(x0, y0), (x1, y1)], fill=(100, 98, 94), width=1)
    img = img.filter(ImageFilter.GaussianBlur(0.5))
    save(img, "concrete.png")


def drywall():
    base = (222, 219, 212)
    img = Image.new("RGB", (N, N), base)
    n = noise_layer(N, 12)
    img = Image.composite(Image.new("RGB", (N, N), tuple(max(0, c - 10) for c in base)),
                           img, n.point(lambda p: 255 if p > 165 else 0))
    draw = ImageDraw.Draw(img)
    panel = N // 4
    for i in range(1, 4):
        draw.line([(i * panel, 0), (i * panel, N)], fill=(195, 192, 185), width=1)
    save(img, "drywall.png")


if __name__ == '__main__':
    wood_plank()
    tile_grid("tile_kitchen.png", (222, 218, 205), grout=(180, 176, 165), tiles=8, speck=8)
    tile_grid("tile_bath.png", (196, 198, 200), grout=(150, 152, 155), tiles=10, speck=10)
    tile_grid("tile_hosp.png", (214, 224, 232), grout=(175, 185, 195), tiles=8, speck=8)
    carpet("carpet_blue.png", (110, 122, 150))
    carpet("carpet_green.png", (118, 148, 126))
    concrete()
    drywall()
