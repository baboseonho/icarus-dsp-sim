"""ICARUS DSP 시뮬레이터. 불러오는 순간 자원 상한(기본 80%)이 걸린다 (dsp/limits.py)."""
from .limits import apply as _apply_limits

_apply_limits()
