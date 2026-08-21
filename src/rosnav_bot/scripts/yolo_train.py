#!/usr/bin/env python3
"""
yolo_train.py — fine-tune Ultralytics YOLO on a local dataset.yaml.

Not a ROS node. Uses the same ultralytics stack as yolo_detector.py:

  # after yolo_collect.py (+ labels)
  python3 src/rosnav_bot/scripts/yolo_train.py \\
      --data ~/yolo_data/cafe/dataset.yaml \\
      --model yolov8n.pt --epochs 50 --imgsz 640

  # smoke (synthetic boxes, 1 epoch)
  python3 src/rosnav_bot/scripts/yolo_train.py --smoke

Deploy the best weights with slam_nav:
  yolo_model:=/path/to/runs/detect/rosnav_yolo/weights/best.pt
"""

from __future__ import annotations

import argparse
import os
import sys


def _make_smoke_dataset(root: str) -> str:
    """Tiny synthetic dataset so --smoke works without Gazebo/labels."""
    import cv2
    import numpy as np

    for split in ('train', 'val'):
        img_dir = os.path.join(root, 'images', split)
        lbl_dir = os.path.join(root, 'labels', split)
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(lbl_dir, exist_ok=True)
        n = 12 if split == 'train' else 4
        for i in range(n):
            img = np.full((320, 320, 3), 40, dtype=np.uint8)
            x1, y1 = 40 + (i % 5) * 20, 50 + (i % 3) * 15
            x2, y2 = x1 + 80, y1 + 60
            color = (20, 180, 40) if i % 2 == 0 else (40, 40, 200)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
            stem = f'smoke_{i:03d}'
            cv2.imwrite(os.path.join(img_dir, f'{stem}.jpg'), img)
            cx = ((x1 + x2) / 2) / 320.0
            cy = ((y1 + y2) / 2) / 320.0
            w = (x2 - x1) / 320.0
            h = (y2 - y1) / 320.0
            cls = i % 2
            with open(os.path.join(lbl_dir, f'{stem}.txt'), 'w', encoding='utf-8') as f:
                f.write(f'{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n')

    yaml_path = os.path.join(root, 'dataset.yaml')
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(
            f'path: {os.path.abspath(root)}\n'
            'train: images/train\n'
            'val: images/val\n'
            'nc: 2\n'
            'names:\n'
            '  0: chair\n'
            '  1: table\n'
        )
    return yaml_path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', default='', help='Path to dataset.yaml')
    ap.add_argument('--model', default='yolov8n.pt', help='Base weights to fine-tune')
    ap.add_argument('--epochs', type=int, default=50)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--batch', type=int, default=8)
    ap.add_argument('--device', default='', help='cuda / cpu / empty=auto')
    ap.add_argument('--project', default='runs/detect')
    ap.add_argument('--name', default='rosnav_yolo')
    ap.add_argument('--smoke', action='store_true',
                    help='Build a tiny synthetic set and train 1 epoch')
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print('ultralytics missing — pip install ultralytics', file=sys.stderr)
        sys.exit(1)

    data = args.data
    epochs = args.epochs
    project = args.project
    name = args.name
    if args.smoke:
        smoke_root = os.path.expanduser('~/yolo_data/_smoke')
        data = _make_smoke_dataset(smoke_root)
        epochs = 1
        project = os.path.abspath('runs/detect')
        name = '_smoke'
        print(f'[yolo_train] smoke dataset → {data}')

    if not data or not os.path.isfile(data):
        print(f'--data required (got {data!r}). Use --smoke for a dry run.',
              file=sys.stderr)
        sys.exit(2)

    model = YOLO(args.model)
    train_kw = dict(
        data=data,
        epochs=epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=project,
        name=name,
        exist_ok=True,
    )
    if args.device:
        train_kw['device'] = args.device

    results = model.train(**train_kw)
    best = getattr(results, 'save_dir', None)
    if best:
        weights = os.path.join(str(best), 'weights', 'best.pt')
        print(f'\n[yolo_train] done. Deploy with:')
        print(f'  ros2 launch rosnav_bot slam_nav.launch.py enable_yolo:=true '
              f'yolo_model:={weights}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
