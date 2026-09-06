"""Inspect user videos inside Kit; video_paths is supplied by the Python server."""

import hashlib
import json
import os
from pathlib import Path

import cv2
import numpy as np

project = Path(os.environ["PANEL_CREASE_PROJECT_ROOT"])
destination = project / "exact_joint" / "references"
destination.mkdir(parents=True,exist_ok=True)
records = []
for filename in video_paths:
    source = Path(filename)
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError("Could not decode " + source.name)
    fps = capture.get(cv2.CAP_PROP_FPS)
    count = capture.get(cv2.CAP_PROP_FRAME_COUNT)
    duration = count/fps
    panels = []
    for fraction in (.02,.20,.40,.60,.80,.98):
        timestamp = duration*fraction
        capture.set(cv2.CAP_PROP_POS_MSEC,timestamp*1000)
        ok,frame = capture.read()
        if not ok:
            raise RuntimeError("Missing video frame")
        scale = min(480/frame.shape[1],340/frame.shape[0])
        resized = cv2.resize(frame,None,fx=scale,fy=scale)
        panel = np.full((375,480,3),240,dtype=np.uint8)
        y,x = (340-resized.shape[0])//2,(480-resized.shape[1])//2
        panel[y:y+resized.shape[0],x:x+resized.shape[1]] = resized
        cv2.putText(panel,f"{timestamp:.2f} s",(12,363),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),1,cv2.LINE_AA)
        panels.append(panel)
    sheet = np.vstack([np.hstack(panels[:3]),np.hstack(panels[3:])])
    output = destination/(source.stem+"_contact_sheet.png")
    cv2.imwrite(str(output),sheet)
    records.append({"file":source.name,"sha256":hashlib.sha256(source.read_bytes()).hexdigest(),
                    "duration_s":duration,"fps":fps,"frame_count":count,"sample_fractions":[.02,.2,.4,.6,.8,.98],
                    "contact_sheet":output.name})
    capture.release()
    print(source.name,duration,output)
(destination/"video_manifest.json").write_text(json.dumps(records,indent=2),encoding="utf-8")
