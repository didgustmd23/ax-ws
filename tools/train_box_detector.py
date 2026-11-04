#!/usr/bin/env python3
"""C1 YOLO11n 학습·독립 test 평가. 데이터와 모델은 로컬 vision_* 폴더에만 저장한다."""
import argparse
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=ROOT / 'vision_data/box_v1/data.yaml')
    parser.add_argument('--output', type=Path, default=ROOT / 'vision_models/box_v1')
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--threads', type=int, default=4)
    args = parser.parse_args()
    if not args.data.is_file() or args.epochs < 1 or args.threads < 1:
        parser.error('Existing dataset and positive epochs/threads required')
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('Output already exists; choose another run name')
    os.environ['YOLO_CONFIG_DIR'] = str(ROOT / 'vision_models/.config')
    import torch
    from ultralytics import YOLO, settings
    # 학습 영상과 지표를 외부 서비스로 자동 전송하지 않는다.
    settings.update({key: False for key in ('sync', 'clearml', 'comet', 'dvc', 'mlflow', 'raytune', 'tensorboard', 'wandb')})
    torch.set_num_threads(args.threads)
    base = ROOT / 'vision_models/yolo11n.pt'
    base.parent.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(base))
    model.train(data=str(args.data.resolve()), epochs=args.epochs, imgsz=args.imgsz,
                batch=8, device='cpu', workers=0, project=str(args.output.parent.resolve()),
                name=args.output.name, exist_ok=False, plots=False, seed=42,
                deterministic=True, amp=False, close_mosaic=5, patience=10)
    best = YOLO(str(args.output / 'weights/best.pt'))
    metrics = best.val(data=str(args.data.resolve()), split='test', imgsz=args.imgsz,
                       device='cpu', workers=0, plots=False, save_json=False,
                       project=str(args.output.parent.resolve()), name=args.output.name + '_test')
    print('Held-out synthetic test: mAP50=', float(metrics.box.map50),
          'mAP50-95=', float(metrics.box.map), 'precision=', float(metrics.box.mp),
          'recall=', float(metrics.box.mr))
    print('Weights:', args.output / 'weights/best.pt')


if __name__ == '__main__':
    main()
