"""Extract labelled, evenly spaced frames for local reference-video inspection."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise ValueError("Cannot open reference video")
    count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = capture.get(cv2.CAP_PROP_FPS)
    tiles = []
    for frame_id in np.linspace(0, count-1, 8).astype(int):
        capture.set(cv2.CAP_PROP_POS_FRAMES, int(frame_id))
        valid, frame = capture.read()
        if not valid:
            continue
        scale = min(380/frame.shape[1], 330/frame.shape[0])
        frame = cv2.resize(frame, None, fx=scale, fy=scale)
        tile = np.full((365, 400, 3), 245, dtype=np.uint8)
        y, x = (330-frame.shape[0])//2, (400-frame.shape[1])//2
        tile[y:y+frame.shape[0], x:x+frame.shape[1]] = frame
        cv2.putText(tile, f"{frame_id/fps:.2f} s", (12, 352),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (20, 20, 20), 1)
        tiles.append(tile)
    capture.release()
    while len(tiles) < 8:
        tiles.append(np.full((365, 400, 3), 245, dtype=np.uint8))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), np.vstack([np.hstack(tiles[:4]), np.hstack(tiles[4:])])):
        raise RuntimeError("Frame sheet could not be written")
    print({"name": args.video.name, "frames": count, "fps": fps, "duration_s": count/fps,
           "sheet": str(args.output)})


if __name__ == "__main__":
    main()
