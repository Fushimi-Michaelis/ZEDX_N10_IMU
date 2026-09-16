#!/usr/bin/env python3
"""Joint capture acceptance: transport and software time mapping, NOT exposure calibration."""
import bisect
import csv
import json
import math
import statistics as st
import sys
from pathlib import Path

root = Path(sys.argv[1])
def read_rows(path):
    with path.open() as f:
        return [{k: v if k=='raw_file' else int(v) for k,v in r.items()} for r in csv.DictReader(f)]
def stats(v):
    v=sorted(v)
    return {'n':len(v),'min':v[0],'median':st.median(v),
            'p95':v[round((len(v)-1)*.95)],'p99':v[round((len(v)-1)*.99)],'max':v[-1]}
imu=read_rows(root/'imu/imu.csv')
ims=json.loads((root/'imu/session.json').read_text())
cams=json.loads((root/'camera/session.json').read_text())
assert ims['boot_id']==cams['boot_id'], 'Different boots: timestamps cannot be joined'
assert not cams['capture_return_code'] and not ims['interrupted']
times=[r['imu_monotonic_estimate_ns'] for r in imu]
assert all(b>a for a,b in zip(times,times[1:])), 'IMU timestamps not strictly increasing'
stable=[r for r in imu if r['imu_monotonic_estimate_ns']>=times[0]+5_000_000_000]
intervals=[b['imu_monotonic_estimate_ns']-a['imu_monotonic_estimate_ns'] for a,b in zip(stable,stable[1:])]
report={'boot_id':ims['boot_id'],'utc_verified':False,'exposure_alignment_verified':False,
        'dynamic_visual_imu_offset_verified':False,
        'meaning':'Nearest estimated IMU sample to VI SOF in CLOCK_MONOTONIC; not measurement of physical synchrony',
        'imu_session':ims,'imu_interval_ns':stats(intervals),
        'imu_frequency_hz':1e9/st.mean(intervals),
        'imu_receive_age_ns':stats([r['receive_monotonic_ns']-r['imu_monotonic_estimate_ns'] for r in stable]),
        'imu_interval_over_7_5ms':sum(v>7_500_000 for v in intervals),
        'accel_norm_mps2':stats([math.sqrt(sum(r[k]**2 for k in ('ax_raw','ay_raw','az_raw')))*12*9.80665/32768 for r in stable]),
        'gyro_mean_dps':{k:st.mean(r[k]*1000/32768 for r in stable) for k in ('gx_raw','gy_raw','gz_raw')},
        'camera':{}}
paired=[]
for side in ('left','right'):
    rows=read_rows(root/f'camera/{side}.csv')
    valid=[r for r in rows if not r['warmup'] and not r['flags']&64 and r['bytesused']==4608000]
    distances=[]; missing=0; converted=[]
    for r in valid:
        # DQBUF contains independent RAW/MONOTONIC samples; keep the per-frame
        # bridge. Its sampling skew is not an exposure-accuracy bound.
        t=r['frame_monotonic_raw_estimate_ns']+r['receive_monotonic_ns']-r['receive_monotonic_raw_ns']
        converted.append(t)
        if not times[0]<=t<=times[-1]:
            missing+=1
            continue
        i=bisect.bisect_left(times,t)
        candidates=[j for j in (i-1,i) if 0<=j<len(times)]
        j=min(candidates,key=lambda j:abs(times[j]-t))
        delta=times[j]-t; distances.append(abs(delta))
        paired.append({'eye':side,'frame_sequence':r['sequence'],'frame_sof_monotonic_estimate_ns':t,
                       'imu_index':imu[j]['index'],'imu_monotonic_estimate_ns':times[j],
                       'imu_minus_sof_ns':delta,'exposure_duration_config_us':10000,
                       'exposure_midpoint_verified':False})
    report['camera'][side]={'frames':len(rows),'post_warmup_frames':len(valid),
        'error_frames':sum(bool(r['flags']&64) for r in rows),
        'sequence_discontinuities':sum(b['sequence']!=a['sequence']+1 for a,b in zip(rows,rows[1:])),
        'uncovered_frames':missing,'nearest_imu_abs_distance_ns':stats(distances),
        'frame_interval_ns':stats([b-a for a,b in zip(converted,converted[1:])])}
assert paired
with (root/'frame-imu-pairs.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(paired[0])); w.writeheader(); w.writerows(paired)
passed=(all(ims[k]==0 for k in ('i2c_errors','ring_drops','fifo_overruns'))
        and report['imu_interval_over_7_5ms']==0 and 195<report['imu_frequency_hz']<205
        and all(r['error_frames']==r['sequence_discontinuities']==r['uncovered_frames']==0
                and r['nearest_imu_abs_distance_ns']['max']<3_000_000 for r in report['camera'].values()))
report['transport_and_estimated_time_pairing_passed']=passed
(root/'joint-report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
if not passed:
    raise SystemExit('Joint capture transport/timestamp acceptance failed; inspect report')
