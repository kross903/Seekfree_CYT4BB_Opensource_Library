import csv, sys
import numpy as np

if len(sys.argv) > 1:
    PATH = sys.argv[1]
else:
    PATH = r'D:/DAS/drone/work/cyt4bb7 case01/Seekfree_CYT4BB_Opensource_Library/vofa+.csv'

rows = []
with open(PATH, 'r') as f:
    r = csv.reader(f); next(r)
    for line in r:
        rows.append([float(x) for x in line])

arr = np.array(rows)
# columns: 0 flow_x_rad, 1 gyro_y_delta, 2 flow_comp_x, 3 vy_raw,
#          4 flow_y_rad, 5 gyro_x_delta, 6 flow_comp_y, 7 vx_raw
fx = arr[:, 0]
gy = arr[:, 1]
cx = arr[:, 2]
fy = arr[:, 4]
gx = arr[:, 5]
cy = arr[:, 6]


def slope(y, x, thr):
    sel = np.abs(x) > thr
    if sel.sum() < 5:
        return float('nan'), 0
    return float(np.dot(x[sel], y[sel]) / np.dot(x[sel], x[sel])), int(sel.sum())


def rms(x):
    return float(np.sqrt((x * x).mean()))


print(f'file: {PATH}')
print(f'rows: {len(rows)}')
print()
print('--- back-out current K from data: K_used = (flow_rad - flow_comp) / gyro_delta ---')
for thr in (0.003, 0.005, 0.008, 0.012):
    sel = np.abs(gy) > thr
    if sel.sum() > 5:
        kx_used = float(np.dot(gy[sel], (fx - cx)[sel]) / np.dot(gy[sel], gy[sel]))
    else:
        kx_used = float('nan')
    sel = np.abs(gx) > thr
    if sel.sum() > 5:
        ky_used = float(np.dot(gx[sel], (fy - cy)[sel]) / np.dot(gx[sel], gx[sel]))
    else:
        ky_used = float('nan')
    print(f'  |gyro|>{thr}: K_x_used={kx_used:.4f}  K_y_used={ky_used:.4f}')

print()
print('--- best-fit K against raw flow (full slope) ---')
for thr in (0.003, 0.005, 0.008, 0.012):
    kx, nx = slope(fx, gy, thr)
    ky, ny = slope(fy, gx, thr)
    print(f'  |gyro|>{thr}: best K_x={kx:.3f}(n={nx})  K_y={ky:.3f}(n={ny})')

print()
print('--- residual slope of flow_comp on gyro (should be ~0 if K is perfect) ---')
for thr in (0.003, 0.005, 0.008, 0.012):
    rkx, _ = slope(cx, gy, thr)
    rky, _ = slope(cy, gx, thr)
    print(f'  |gyro|>{thr}: residual slope cx~gy={rkx:+.3f}  cy~gx={rky:+.3f}')

print()
print('--- RMS sweep on this dataset (apply candidate K to RAW flow & gyro) ---')
print('    NOTE: this assumes data was logged with K=1.0; if not, see "K_used" above.')
for kxg, kyg in [(1.00, 1.00), (1.30, 1.20), (1.43, 1.32), (1.50, 1.40), (1.60, 1.50)]:
    print(f'  K_X={kxg:.2f} K_Y={kyg:.2f}  RMS_comp_x={rms(fx - kxg * gy):.5f}  '
          f'RMS_comp_y={rms(fy - kyg * gx):.5f}')
print(f'  baseline (K=1.0)        : RMS_comp_x={rms(fx - gy):.5f}  RMS_comp_y={rms(fy - gx):.5f}')

print()
print('--- direct stats of logged flow_comp ---')
print(f'  RMS  flow_comp_x={rms(cx):.5f}  flow_comp_y={rms(cy):.5f}')
print(f'  mean flow_comp_x={cx.mean():+.5f}  flow_comp_y={cy.mean():+.5f}')
print(f'  std  flow_comp_x={cx.std():.5f}  flow_comp_y={cy.std():.5f}')

print()
print('--- per-segment best-fit K ---')
N = arr.shape[0]
seg_n = 6
seg = N // seg_n
print(f'{"seg":>3} {"nX":>4} {"K_X":>7} {"nY":>4} {"K_Y":>7} {"rmsCx":>9} {"rmsCy":>9}')
for i in range(seg_n):
    s = arr[i * seg:(i + 1) * seg]
    fxs = s[:, 0]; gys = s[:, 1]; fys = s[:, 4]; gxs = s[:, 5]
    cxs = s[:, 2]; cys = s[:, 6]
    kxs, nxs = slope(fxs, gys, 0.005)
    kys, nys = slope(fys, gxs, 0.005)
    print(f'{i:>3} {nxs:>4} {kxs:7.3f} {nys:>4} {kys:7.3f} {rms(cxs):9.5f} {rms(cys):9.5f}')
