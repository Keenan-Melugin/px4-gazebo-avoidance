#!/usr/bin/env python3
"""What happened in a run, from PX4's own flight log.

    pip install pyulog
    python3 scripts/ulog_timeline.py ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/<date>/<time>.ulg

Prints, every few seconds of the log: position, flight mode, arming, and
the nearest range in the fifteen histogram bins ahead of the aircraft as PX4
received them. Before that, the gaps in the two streams this stack lives
on: the synthetic sticks (PX4 declares RC loss after 0.5 s without them) and
the obstacle histogram (collision prevention treats it as no data after
0.5 s). A stall of the ROS side shows up here as a gap in both.

This is the script that explained a brake test which ended 50 m from where
it should have (docs/data.md): the aircraft braked at 2.0 m, then crept along
the wall toward free space while the stick kept pushing. The log said so in
one screen; the test's own output could not.
"""
import argparse
import os

import numpy as np
from pyulog import ULog

AHEAD = list(range(65, 72)) + list(range(0, 8))     # the camera's 15 bins

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument('ulg')
ap.add_argument('--step', type=float, default=5.0, help='seconds per printed row')
a = ap.parse_args()

u = ULog(a.ulg, message_name_filter_list=[
    'vehicle_local_position', 'manual_control_setpoint', 'obstacle_distance',
    'vehicle_status'])
d = {m.name: m for m in u.data_list}
missing = [m for m in ('vehicle_local_position', 'manual_control_setpoint',
                       'obstacle_distance', 'vehicle_status') if m not in d]
if missing:
    raise SystemExit('log has no %s; is it from this stack?' % ', '.join(missing))


def col(name, field):
    return d[name].data['timestamp'] / 1e6, np.asarray(d[name].data[field])


t_pos, x = col('vehicle_local_position', 'x')
_, y = col('vehicle_local_position', 'y')
_, z = col('vehicle_local_position', 'z')
t_mc, _ = col('manual_control_setpoint', 'timestamp')
t_od, _ = col('obstacle_distance', 'timestamp')
od_bins = np.column_stack([d['obstacle_distance'].data['distances[%d]' % i] for i in range(72)])
t_st, nav = col('vehicle_status', 'nav_state')
_, arm = col('vehicle_status', 'arming_state')

print('%s: %.0f s, %d position samples, %d sticks, %d histograms'
      % (os.path.basename(a.ulg), t_pos[-1], len(t_pos), len(t_mc), len(t_od)))
for label, t, limit in (('stick stream', t_mc, 0.3), ('histogram stream', t_od, 0.5)):
    gaps = np.diff(t)
    big = np.where(gaps > limit)[0]
    print('%s: median gap %.3f s, max gap %.2f s, gaps over %.1f s: %d'
          % (label, np.median(gaps), gaps.max(), limit, len(big)))
    for i in big[:8]:
        print('    silent %.2f s at t=%.1f' % (gaps[i], t[i]))

print()
print('  t(s)  north   east    alt  nav arm | ahead min(cm) | note')
for t in np.arange(np.floor(t_pos[0]), t_pos[-1], a.step):
    i = np.searchsorted(t_pos, t)
    if i >= len(t_pos):
        break
    j = min(np.searchsorted(t_st, t), len(t_st) - 1)
    k = np.searchsorted(t_od, t)
    if k < len(t_od):
        amin = int(od_bins[k, AHEAD].min())
        note = ('clear' if amin >= 1911 else
                'UNKNOWN' if amin == 65535 else 'wall %.1f m' % (amin / 100.0))
        age = t - t_od[k - 1] if k > 0 else 0.0
        if age > 0.5:
            note += ' (histogram %.1f s old)' % age
    else:
        amin, note = -1, 'no histogram'
    print('%6.0f %6.2f %6.2f %6.2f  %2d  %2d | %7d | %s'
          % (t, x[i], y[i], -z[i], nav[j], arm[j], amin, note))
