#!/usr/bin/env python3
# Plots for the fillet oracle (docs/fillet-oracle.md). Reads the --out directory of tools/fillet_oracle.mjs.
#   python3 -I tools/fillet_oracle_plot.py DATA_DIR OUT_DIR
import json, sys, os, glob
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource

data, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)

def load(case, kind, r):
    tag = f'{case}_{kind}_r{r}'
    p = os.path.join(data, tag + '.json')
    if not os.path.exists(p):
        return None
    m = json.load(open(p))
    nx, ny = m['nx'], m['ny']
    zo = np.fromfile(os.path.join(data, tag + '_oracle.f32'), dtype=np.float32).reshape(ny, nx)
    zk = np.fromfile(os.path.join(data, tag + '_kernel.f32'), dtype=np.float32).reshape(ny, nx)
    m['zs'] = np.fromfile(os.path.join(data, tag + '_swept.f32'), dtype=np.float32).reshape(ny, nx)
    return m, zo, zk

def extent(m):
    return [m['x0'], m['x0'] + m['nx'] * m['cell'], m['y0'], m['y0'] + m['ny'] * m['cell']]

def outline(ax, case):
    segs = json.load(open(os.path.join(data, f'{case}_profile.json')))
    for s in segs:
        ax.plot([s[0], s[2]], [s[1], s[3]], 'k-', lw=0.6)

def fmt(r):
    return str(r).replace('.0', '') if float(r).is_integer() else str(r)

def three_panel(case, kind, r):
    d = load(case, kind, fmt(r))
    if d is None:
        return None
    m, zo, zk = d
    ext = extent(m)
    zs = m['zs']
    fig, axs = plt.subplots(2, 2, figsize=(15, 9), constrained_layout=True); axs = axs.ravel()
    vmin = np.nanmin(zo); vmax = m['H']
    for ax, z, t in ((axs[0], zo, 'oracle (opening)'), (axs[1], zk, 'kernel')):
        im = ax.imshow(z, origin='lower', extent=ext, vmin=vmin, vmax=vmax, cmap='viridis')
        outline(ax, case); ax.set_title(f'{t} top height, {kind} {r} (mm)'); ax.set_aspect('equal')
        fig.colorbar(im, ax=ax, shrink=0.8, label='z (mm)')
    for ax, ref, t in ((axs[2], zo, 'kernel - oracle (opening)'), (axs[3], zs, 'kernel - oracle (swept-cut)')):
        diff = zk - ref
        lim = max(0.05, float(np.nanpercentile(np.abs(diff), 99.5))) if np.isfinite(diff).any() else 0.05
        im = ax.imshow(diff, origin='lower', extent=ext, vmin=-lim, vmax=lim, cmap='RdBu_r')
        outline(ax, case); ax.set_aspect('equal'); ax.set_title(f'{t} (mm), colour range +-{lim:.2f}')
        fig.colorbar(im, ax=ax, shrink=0.8, label='mm')
    f = os.path.join(out, f'{case}_{kind}_r{fmt(r)}_maps.png'); fig.savefig(f, dpi=80); plt.close(fig); return f

def hillshade(case, kind, r):
    d = load(case, kind, fmt(r))
    if d is None:
        return None
    m, zo, zk = d
    ext = extent(m); z = np.where(np.isnan(zo), np.nanmin(zo), zo)
    ls = LightSource(azdeg=315, altdeg=40)
    rgb = ls.shade(z, cmap=plt.cm.viridis, vert_exag=3, dx=m['cell'], dy=m['cell'], blend_mode='soft')
    rgb[np.isnan(zo)] = 1
    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    ax.imshow(rgb, origin='lower', extent=ext); outline(ax, case); ax.set_aspect('equal')
    ax.set_title(f'oracle surface (lit, z x3), {kind} {r}')
    f = os.path.join(out, f'{case}_{kind}_r{fmt(r)}_hillshade.png'); fig.savefig(f, dpi=100); plt.close(fig); return f

def surf3d(case, kind, r):
    d = load(case, kind, fmt(r))
    if d is None:
        return None
    m, zo, zk = d
    st = max(1, m['nx'] // 260)
    x = m['x0'] + (np.arange(m['nx']) + 0.5) * m['cell']; y = m['y0'] + (np.arange(m['ny']) + 0.5) * m['cell']
    X, Y = np.meshgrid(x[::st], y[::st]); Z = zo[::st, ::st]
    fig = plt.figure(figsize=(12, 6)); ax = fig.add_subplot(111, projection='3d')
    ls = LightSource(azdeg=300, altdeg=45)
    Zm = np.ma.masked_invalid(Z); rgb = ls.shade(np.nan_to_num(Z, nan=0), cmap=plt.cm.viridis, vert_exag=2, blend_mode='soft')
    ax.plot_surface(X, Y, Zm, facecolors=rgb, rstride=1, cstride=1, linewidth=0, antialiased=False, shade=False)
    ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(y) * 0.6)); ax.view_init(35, -60); ax.set_zlim(0, m['H'] * 1.05)
    ax.set_title(f'oracle surface, {case} {kind} {r}')
    f = os.path.join(out, f'{case}_{kind}_r{fmt(r)}_3d.png'); fig.savefig(f, dpi=100); plt.close(fig); return f

def sections(case, kind, rs, lines):
    fig, axs = plt.subplots(len(rs), len(lines), figsize=(6.2 * len(lines), 2.6 * len(rs)), constrained_layout=True, squeeze=False)
    for i, r in enumerate(rs):
        d = load(case, kind, fmt(r))
        for j, (name, axis, v) in enumerate(lines):
            ax = axs[i][j]
            if d is None:
                ax.set_title(f'{name}, r={r}: kernel did not build'); continue
            m, zo, zk = d
            xs = m['x0'] + (np.arange(m['nx']) + 0.5) * m['cell']; ys = m['y0'] + (np.arange(m['ny']) + 0.5) * m['cell']
            if axis == 'y':
                jj = int(round((v - m['y0']) / m['cell'] - 0.5)); co, a, b, c3 = xs, zo[jj, :], zk[jj, :], m['zs'][jj, :]; lab = 'x'
            else:
                ii = int(round((v - m['x0']) / m['cell'] - 0.5)); co, a, b, c3 = ys, zo[:, ii], zk[:, ii], m['zs'][:, ii]; lab = 'y'
            ax.plot(co, a, 'b-', lw=1.6, label='oracle'); ax.plot(co, c3, 'g-.', lw=1.2, label='oracle swept-cut'); ax.plot(co, b, 'r--', lw=1.2, label='kernel')
            ax.set_title(f'{name} ({"y" if axis == "y" else "x"}={v}), r={r}'); ax.set_xlabel(lab + ' (mm)'); ax.set_ylabel('z (mm)')
            if i == 0 and j == 0: ax.legend()
    f = os.path.join(out, f'{case}_{kind}_sections.png'); fig.savefig(f, dpi=100); plt.close(fig); return f

def errplot():
    fig, ax = plt.subplots(figsize=(9, 5), constrained_layout=True)
    for rows_f in sorted(glob.glob(os.path.join(data, 'rows_*.json'))):
        rows = json.load(open(rows_f))['rows']
        for case in sorted({r['case'] for r in rows}):
            for kind in ('round', 'chamfer'):
                rr = [r for r in rows if r['case'] == case and r['kind'] == kind and r.get('status') == 'built']
                if not rr: continue
                l, = ax.plot([r['r'] for r in rr], [r['maxFlat'] for r in rr], 'o-', label=f'{case} {kind} vs opening, max')
                ax.plot([r['r'] for r in rr], [r['rmsFlat'] for r in rr], 's:', color=l.get_color(), label=f'{case} {kind} vs opening, rms')
                if 'sweptMaxFlat' in rr[0]:
                    l2, = ax.plot([r['r'] for r in rr], [r['sweptMaxFlat'] for r in rr], '^--', label=f'{case} {kind} vs swept-cut, max')
                    ax.plot([r['r'] for r in rr], [r['sweptRmsFlat'] for r in rr], 'v:', color=l2.get_color(), label=f'{case} {kind} vs swept-cut, rms')
    ax.set_xlabel('radius / distance r (mm)'); ax.set_ylabel('height error vs oracle (mm)'); ax.set_yscale('log'); ax.grid(alpha=.3); ax.legend(fontsize=7)
    ax.set_title('kernel top height minus oracle (cells where the oracle slope < 2)')
    f = os.path.join(out, 'error_vs_radius.png'); fig.savefig(f, dpi=100); plt.close(fig); return f

made = []
for case, radii, lines in (('blob', (3.7, 15, 24.9), [('through the left tip', 'y', 13.5), ('mid wall', 'x', -20)]),
                           ('d', (2, 5, 7), [('through a corner', 'y', 0.5), ('mid arc', 'x', 0)])):
    for r in radii:
        for fn in (three_panel, hillshade, surf3d):
            f = fn(case, 'round', r)
            if f: made.append(f)
    if os.path.exists(os.path.join(data, f'{case}_round_r{fmt(radii[0])}.json')):
        made.append(sections(case, 'round', list(radii), lines))
made.append(errplot())
print('\n'.join(made))
