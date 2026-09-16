#!/usr/bin/env python3
"""Export stereo PNG/RAW pairs selected by V4L2 hardware timestamps, not indices."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from zed_raw_processing import load_profile, shared_white_level, render

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('capture_dir', type=Path)
p.add_argument('output_dir', type=Path)
p.add_argument('--pairs', type=int, default=5)
p.add_argument('--tolerance-us', type=int, default=1000)
p.add_argument('--profile', type=Path, default=Path(__file__).with_name('zedx_color_profile.json'))
a = p.parse_args()
if not 1 <= a.pairs <= 20 or not 0 < a.tolerance_us <= 10000:
    p.error('Pairs must be 1..20 and tolerance 1..10000 microseconds')
session = json.loads((a.capture_dir / 'session.json').read_text())
if session['capture_return_code']:
    raise SystemExit('Capture did not complete successfully')
profile = load_profile(a.profile)
def read_rows(side):
    with (a.capture_dir / f'{side}.csv').open() as f:
        rows = [{k: (v if k == 'raw_file' else int(v)) for k, v in row.items()}
                for row in csv.DictReader(f)]
    stamps = [r['v4l2_timestamp_ns'] for r in rows]
    if any(x <= 0 for x in stamps) or any(b <= x for x, b in zip(stamps, stamps[1:])):
        raise ValueError('Invalid or non-monotonic frame timestamps')
    return rows
all_rows = {side: read_rows(side) for side in ('left', 'right')}
valid = {side: [r for r in rows if not r['warmup'] and not r['flags'] & 0x40 and r['bytesused'] == 4608000]
         for side, rows in all_rows.items()}
left, right = valid['left'], valid['right']
if not left or not right:
    raise SystemExit('No valid post-warmup frames')
pairs, j = [], 0
for l in left:
    if j >= len(right):
        break
    t = l['v4l2_timestamp_ns']
    while j + 1 < len(right) and abs(right[j+1]['v4l2_timestamp_ns']-t) < abs(right[j]['v4l2_timestamp_ns']-t):
        j += 1
    r = right[j]
    if abs(r['v4l2_timestamp_ns']-t) <= a.tolerance_us * 1000:
        pairs.append((l, r))
        j += 1
saved = [(l, r) for l, r in pairs if l['raw_file'] and r['raw_file']]
if len(saved) < a.pairs:
    raise SystemExit(f'Only {len(saved)} saved, time-matched pairs; requested {a.pairs}')
selected = saved[:a.pairs]
def stats(values):
    q = np.asarray(values, dtype=np.float64)
    return {'min': float(q.min()), 'median': float(np.median(q)),
            'p95': float(np.percentile(q, 95)), 'max': float(q.max())}
report = {
    'session': session, 'timestamp_domain': 'RTCPU/architected hardware counter; validated with CNTVCT samples',
    'timestamp_event': 'VI SOF arrival according to vendor implementation/documentation; V4L2 flags advertise EOF',
    'not_exposure_or_cpld_timestamp': True, 'utc_calibrated': False,
    'pair_tolerance_us': a.tolerance_us, 'matched_post_warmup_pairs': len(pairs),
    'absolute_pair_delta_us': stats([abs(r['v4l2_timestamp_ns']-l['v4l2_timestamp_ns'])/1000 for l,r in pairs]),
    'exported_pairs': a.pairs, 'color_profile': profile, 'eyes': {},
}
overlap_lo=max(left[0]['v4l2_timestamp_ns'], right[0]['v4l2_timestamp_ns'])
overlap_hi=min(left[-1]['v4l2_timestamp_ns'], right[-1]['v4l2_timestamp_ns'])
interior_unmatched = {}
for i, side in enumerate(('left','right')):
    rows, rows_valid = all_rows[side], valid[side]
    intervals = np.diff([r['v4l2_timestamp_ns'] for r in rows_valid])/1e6
    matched_ids={pair[i]['index'] for pair in pairs}
    interior_unmatched[side]=sum(r['index'] not in matched_ids and
        overlap_lo+a.tolerance_us*1000 < r['v4l2_timestamp_ns'] < overlap_hi-a.tolerance_us*1000 for r in rows_valid)
    report['eyes'][side] = {
        'captured_frames':len(rows), 'valid_post_warmup_frames':len(rows_valid),
        'v4l2_error_frames':sum(bool(r['flags'] & 0x40) for r in rows),
        'sequence_discontinuities':sum(b['sequence'] != x['sequence']+1 for x,b in zip(rows,rows[1:])),
        'interval_ms':stats(intervals), 'median_fps':float(1000/np.median(intervals)),
        'unmatched_post_warmup_frames':len(rows_valid)-len(pairs),
        'unmatched_inside_overlap':interior_unmatched[side],
        'dequeue_latency_ms_estimate':stats([(r['receive_monotonic_raw_ns']-r['frame_monotonic_raw_estimate_ns'])/1e6 for r in rows_valid]),
        'counter_to_monotonic_raw_offset_ns':stats([r['counter_to_raw_offset_ns'] for r in rows_valid]),
        'clock_sample_span_ns':stats([r['clock_sample_span_ns'] for r in rows_valid]),
    }
report['all_interior_frames_paired']=not any(interior_unmatched.values())
if not report['all_interior_frames_paired']:
    raise SystemExit('Unpaired frames inside overlap; synchronization not verified')

a.output_dir.mkdir(parents=True, exist_ok=True)
payloads = {}
for side, i in [('left',0),('right',1)]:
    payloads[side] = []
    for pair in selected:
        blob=(a.capture_dir / pair[i]['raw_file']).read_bytes()
        if len(blob)!=4608000:
            raise ValueError('Saved RAW length mismatch')
        payloads[side].append(blob)
frames={s:[np.frombuffer(b,dtype='<u2').reshape(1200,1920) for b in blobs] for s,blobs in payloads.items()}
white=shared_white_level([frames['left'][-1],frames['right'][-1]],profile)
manifest=[]
for n, pair in enumerate(selected, 1):
    record={'pair':n,'right_minus_left_us':(pair[1]['v4l2_timestamp_ns']-pair[0]['v4l2_timestamp_ns'])/1000,
            'boot_id':session['boot_id'], 'timestamp_precision_ns':session['timestamp_precision_ns'],
            'system_clock_synchronized':session['system_clock_synchronized'], 'exposure_alignment_optically_measured':False}
    for i, side in enumerate(('left','right')):
        folder=a.output_dir / 'images' / side; folder.mkdir(parents=True, exist_ok=True)
        image_path=folder / f'{side}_{n:02d}.png'
        image=render(frames[side][n-1],profile,white)
        if not cv2.imwrite(str(image_path),image): raise OSError('PNG write failed')
        item=dict(pair[i])
        item['source_capture_raw_file']=item.pop('raw_file')
        item['png_file']=f'images/{side}/{image_path.name}'
        item['png_sha256']=hashlib.sha256(image_path.read_bytes()).hexdigest()
        item['raw_frame_sha256']=hashlib.sha256(payloads[side][n-1]).hexdigest()
        item['raw_batch_file']=f'raw/{side}_5frames.raw'
        item['raw_batch_byte_offset']=(n-1)*4608000
        item['receive_realtime_is_utc_trusted']=False
        record[side]=item
        (folder / f'{side}_{n:02d}.json').write_text(json.dumps(item,indent=2)+'\n')
    manifest.append(record)
(a.output_dir / 'raw').mkdir(exist_ok=True)
for side in ('left','right'):
    (a.output_dir / 'raw' / f'{side}_5frames.raw').write_bytes(b''.join(payloads[side]))
last=[render(frames[s][-1],profile,white) for s in ('left','right')]
cv2.imwrite(str(a.output_dir / 'stereo-preview.png'), np.hstack([cv2.resize(im,(960,600)) for im in last]))
cv2.imwrite(str(a.output_dir / 'images' / 'stereo_preview.png'), np.hstack([cv2.resize(im,(960,600)) for im in last]))
for name,data in [('pairs.json',manifest),('timing_report.json',report),('session.json',session)]:
    (a.output_dir / name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
with (a.output_dir / 'pairs.csv').open('w',newline='') as f:
    writer=csv.writer(f)
    writer.writerow(['pair','left_png','right_png','left_sequence','right_sequence','left_hw_timestamp_ns','right_hw_timestamp_ns','right_minus_left_us','left_frame_monotonic_raw_ns_estimate','right_frame_monotonic_raw_ns_estimate','left_raw_frame_sha256','right_raw_frame_sha256'])
    for r in manifest:
        l,v=r['left'],r['right']
        writer.writerow([r['pair'],l['png_file'],v['png_file'],l['sequence'],v['sequence'],l['v4l2_timestamp_ns'],v['v4l2_timestamp_ns'],r['right_minus_left_us'],l['frame_monotonic_raw_estimate_ns'],v['frame_monotonic_raw_estimate_ns'],l['raw_frame_sha256'],v['raw_frame_sha256']])
for side in ('left','right'):
    (a.output_dir / f'{side}-all-timestamps.csv').write_bytes((a.capture_dir / f'{side}.csv').read_bytes())
print(json.dumps({'matched_pairs':len(pairs),'exported_pairs':a.pairs,
    'delta_us':report['absolute_pair_delta_us'],'utc_calibrated':report['utc_calibrated'],
    'median_fps':{s:report['eyes'][s]['median_fps'] for s in ('left','right')}},indent=2))
