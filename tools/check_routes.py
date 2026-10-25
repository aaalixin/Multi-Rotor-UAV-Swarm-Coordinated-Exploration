#!/usr/bin/env python3
"""校验 simple_explore.cpp cfgs 与 vision_track.py ROUTES 完全一致 (航点+sox+soy+spd)"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CPP = os.path.join(ROOT, 'src/drone_ctrl/src/simple_explore.cpp')
PY = os.path.join(ROOT, 'src/vision_track.py')

cpp_routes = {}
cur = None
dcount = 0
entry_re = re.compile(r'^\s*\{\{((?:\{-?[\d.]+,-?[\d.]+\},?)+)\},(-?[\d.]+),(-?[\d.]+),([\d.]+)\},\s*$')
for ln in open(CPP):
    m = re.search(r'//\s*route\s+(\d+)', ln)
    if m:
        cur = int(m.group(1))
        dcount = 0
        continue
    m = entry_re.match(ln)
    if m and cur is not None:
        pairs = re.findall(r'\{(-?[\d.]+),(-?[\d.]+)\}', m.group(1))
        wps = tuple((float(a), float(b)) for a, b in pairs)
        cpp_routes[(cur, dcount)] = (wps, float(m.group(2)), float(m.group(3)), float(m.group(4)))
        dcount += 1

py_routes = {}
cur = None
py_entry_re = re.compile(r'^\s*(\d+):\s*\(densify\(\[(.*?)\]\),\s*(-?[\d.]+),\s*(-?[\d.]+),\s*([\d.]+)\),\s*$')
for ln in open(PY):
    m = re.search(r'#\s*route\s+(\d+)', ln)
    if m:
        cur = int(m.group(1))
        continue
    m = py_entry_re.match(ln)
    if m and cur is not None:
        d = int(m.group(1))
        pairs = re.findall(r'\((-?[\d.]+),\s*(-?[\d.]+)\)', m.group(2))
        wps = tuple((float(a), float(b)) for a, b in pairs)
        py_routes[(cur, d)] = (wps, float(m.group(3)), float(m.group(4)), float(m.group(5)))

bad = 0
for k in sorted(set(cpp_routes) | set(py_routes)):
    a, b = cpp_routes.get(k), py_routes.get(k)
    if a is None or b is None:
        print('MISSING', k, 'in', 'cpp' if a is None else 'py')
        bad += 1
        continue
    if a != b:
        print('DIFF', k)
        print('  cpp', a)
        print('  py ', b)
        bad += 1

print('cpp entries:', len(cpp_routes), '| py entries:', len(py_routes))
assert len(cpp_routes) == 42 and len(py_routes) == 42, '期望42条'
print('两文件完全一致' if bad == 0 else '不一致: %d 条' % bad)
sys.exit(1 if bad else 0)
