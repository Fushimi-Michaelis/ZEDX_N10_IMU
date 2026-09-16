#!/usr/bin/env python3
"""Decode captured BA10/unpacked-16 Bayer frames into diagnostic previews."""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
from zed_raw_processing import load_frames, load_profile, shared_white_level, render, raw_path

root = Path(sys.argv[1])
profile_path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).with_name('zedx_color_profile.json')
profile = load_profile(profile_path)
captures = {d: load_frames(raw_path(root, d)) for d in (4, 5)}
white = shared_white_level([v[-1] for v in captures.values()], profile)
views, report = [], {'color_processing': {'bayer': 'GRBG', 'rgb_gains': profile['rgb_gains'],
    'black_level_raw10': profile['black_level_raw10'], 'shared_white_level': white,
    'transfer': 'sRGB', 'profile': str(profile_path)}}
for device in (4, 5):
    filename = raw_path(root, device)
    frames = captures[device]
    frame = frames[-1]
    low, high = np.percentile(frame, (0.1, 99.9))
    report[str(device)] = {
        'frames': len(frames), 'bytes': filename.stat().st_size,
        'min': int(frame.min()), 'max': int(frame.max()),
        'nonzero_fraction': float(np.count_nonzero(frame) / frame.size),
        'preview_levels': [float(low), float(high)],
        'mean_absolute_change': float(np.mean(np.abs(frames[-1].astype(float) - frames[0]))),
    }
    if high <= low:
        raise SystemExit('Flat frame: not a valid scene capture')
    color = render(frame, profile, white)
    gray8 = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(str(root / f'video{device}-gray.png'), gray8)
    cv2.imwrite(str(root / f'video{device}-color.png'), color)
    views.append(cv2.resize(color, (960, 600)))
cv2.imwrite(str(root / 'stereo-preview.png'), np.hstack(views))
(root / 'capture-stats.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
