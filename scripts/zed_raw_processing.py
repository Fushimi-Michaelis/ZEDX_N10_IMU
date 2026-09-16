"""N10 BA10/T_R16 processing: explicit GRBG, linear WB, shared exposure, sRGB."""
import json
from pathlib import Path
import cv2
import numpy as np

def raw_path(root, device):
    root = Path(root)
    current = root / 'raw' / ('left_5frames.raw' if device == 4 else 'right_5frames.raw')
    return current if current.exists() else root / f'video{device}-first.raw'

def load_frames(filename):
    data = np.fromfile(filename, dtype='<u2')
    if not data.size or data.size % (1920 * 1200):
        raise ValueError(f'Invalid capture size: {filename}')
    frames = data.reshape(-1, 1200, 1920)
    return frames

def linear_bgr(raw16, black_level=42):
    # N10 capture uses expanded RAW10: low bits repeat the high RAW10 bits.
    raw10 = raw16 >> 6
    if not np.array_equal(raw16, (raw10 << 6) | (raw10 >> 4)):
        raise ValueError('Unexpected RAW packing; this decoder requires N10 BA10/T_R16')
    # Do not use COLOR_BayerGR2BGR: that short alias reverses R/B for GRBG.
    color = cv2.cvtColor(raw10, cv2.COLOR_BayerGRBG2BGR_EA).astype(np.float32)
    return np.maximum(color - float(black_level), 0)

def load_profile(path):
    profile = json.loads(Path(path).read_text())
    if profile.get('bayer_pattern') != 'GRBG':
        raise ValueError('This sensor/format is GRBG; other Bayer patterns are not supported')
    black = float(profile['black_level_raw10'])
    if not np.isfinite(black) or not 0 <= black < 1023:
        raise ValueError('Black level must be in [0,1023) RAW10 counts')
    gains = np.asarray(profile['rgb_gains'], dtype=float)
    if gains.shape != (3,) or not np.all(np.isfinite(gains)) or np.any(gains <= 0) or np.any(gains > 8):
        raise ValueError('RGB white-balance gains must be positive and at most 8')
    return profile

def balance(linear, profile):
    return linear * np.array(profile['rgb_gains'][::-1], dtype=np.float32)

def shared_white_level(last_frames, profile):
    samples = [balance(linear_bgr(frame, profile['black_level_raw10']), profile)[::4, ::4].max(axis=2)
               for frame in last_frames]
    value = float(np.percentile(np.concatenate([s.ravel() for s in samples]), 99.8))
    if value <= 0:
        raise ValueError('No usable image signal')
    return value

def render(raw16, profile, white_level):
    linear = balance(linear_bgr(raw16, profile['black_level_raw10']), profile)
    linear = np.clip(linear / white_level, 0, 1)
    srgb = np.where(linear <= 0.0031308, 12.92 * linear,
                    1.055 * np.power(linear, 1 / 2.4) - 0.055)
    return np.rint(srgb * 255).astype(np.uint8)
