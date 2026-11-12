"""check.py들이 함께 쓰는 채점 도구. 응답 지표를 계산하고 PASS/FAIL 표를 찍는다.

Grading helpers shared by every check.py: response metrics and the PASS/FAIL table.

    rep = Report("week01")
    rep.check("hover thrust", ok, value="0.372 N", target="m·g",
              hint=("오차 부호가 반대예요. e = 목표 - 현재", "The sign is flipped: e = target - current."))
    rep.done()        # 요약 출력, 결과 json 저장, 종료 코드
"""
from __future__ import annotations

import json
import os
import sys
import traceback
from pathlib import Path

import numpy as np

GREEN, RED, YEL, DIM, BOLD, END = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[1m", "\033[0m"
if not sys.stdout.isatty() and not os.environ.get("FORCE_COLOR"):
    GREEN = RED = YEL = DIM = BOLD = END = ""


# 지표
def step_metrics(t, y, t_step, y0, y1, band=0.02):
    """계단 응답 지표. 오버슈트 [%], 2% 정착 시간 [s], 정상상태 오차 [m]를 돌려준다.
    정상상태 오차는 마지막 1초 평균으로 잰다.

    Step-response metrics: overshoot [%], 2% settling time [s] and
    steady-state error [m], the last one averaged over the final second.
    """
    t, y = np.asarray(t), np.asarray(y)
    m = t >= t_step
    tt, yy = t[m] - t_step, y[m]
    amp = y1 - y0
    overshoot = max(0.0, (np.max((yy - y0) / amp) - 1.0) * 100.0)
    tol = band * abs(amp)
    outside = np.where(np.abs(yy - y1) > tol)[0]
    settle = 0.0 if len(outside) == 0 else (tt[outside[-1] + 1] if outside[-1] + 1 < len(tt) else np.inf)
    ss = abs(np.mean(yy[tt >= tt[-1] - 1.0]) - y1)
    return {"overshoot_pct": float(overshoot), "settle_s": float(settle), "ss_err_m": float(ss)}


def rms(x, axis=None):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(np.sum(x ** 2, axis=-1)))) if axis is None else np.sqrt(np.mean(x ** 2, axis=axis))


def formation_error(P, offsets):
    """대형 오차의 RMS. 대형 전체가 평행이동한 것은 오차로 치지 않는다.
    P (..., n, 3)와 offsets (n, 3)를 받아 (...,)를 돌려준다.

    RMS formation error, ignoring a translation of the whole formation.
    """
    P = np.asarray(P, float); O = np.asarray(offsets, float)
    X = P - O
    X = X - X.mean(axis=-2, keepdims=True)
    return np.sqrt(np.mean(np.sum(X ** 2, axis=-1), axis=-1))


def pair_distances(P):
    """모든 쌍의 거리. (..., n, 3)을 받아 (..., n, n)을 돌려준다.

    Pairwise distances, (..., n, 3) to (..., n, n).
    """
    P = np.asarray(P, float)
    d = P[..., :, None, :] - P[..., None, :, :]
    return np.linalg.norm(d, axis=-1)


def min_pair_distance(P):
    """시각마다 가장 가까운 두 기체 사이의 거리.

    Minimum inter-drone distance at each time step.
    """
    D = pair_distances(P)
    n = D.shape[-1]
    iu = np.triu_indices(n, 1)
    return D[..., iu[0], iu[1]].min(axis=-1)


# 결과 표

class Report:
    """시험 결과를 한 줄씩 찍고, 끝에 요약과 json을 남긴다.

    Collects test results, prints one line per check, and writes a summary
    and a json file at the end.
    """

    def __init__(self, name, out_dir=None):
        self.name = name
        self.rows = []
        self.out_dir = Path(out_dir) if out_dir else None
        self.extra = {}
        self.in_advanced = False

    def advanced(self):
        """여기부터는 심화 시험. 심화는 요약에 따로 세고, 종료 코드는 핵심만으로 정한다.

        From here on the checks belong to the advanced part. They are counted separately
        and do not decide the exit code (unless DSP_STRICT=1).
        """
        self.in_advanced = True

    def check(self, label, ok, value="", target="", hint=None):
        ok = bool(ok)
        self.rows.append({"label": label, "ok": ok, "value": str(value), "target": str(target),
                          "hint": None if ok or hint is None else list(hint), "advanced": self.in_advanced})
        tag = f"{GREEN}PASS{END}" if ok else f"{RED}FAIL{END}"
        print(f"  {tag}  {label:<46s} {value:>18s}   {DIM}기준 {target}{END}")
        if not ok and hint:
            print(f"        {YEL}힌트: {hint[0]}\n        Hint: {hint[1]}{END}")
        return ok

    def error(self, label, exc: BaseException, hint=None):
        msg = f"{type(exc).__name__}: {exc}"
        tb = traceback.extract_tb(exc.__traceback__)
        where = next((f"{Path(f.filename).name}:{f.lineno}" for f in reversed(tb) if "exercise" in f.filename), "")
        if isinstance(exc, NotImplementedError):
            hint = hint or ("아직 이 TODO를 채우지 않았어요.", "This TODO is not filled in yet.")
        self.check(label, False, value="error", target=where or "-",
                   hint=hint or (f"코드를 실행하다 오류가 났어요. {msg}", f"Your code raised an error. {msg}"))

    @property
    def passed(self):
        """핵심 시험이 모두 통과했나."""
        core = [r for r in self.rows if not r["advanced"]]
        return len(core) > 0 and all(r["ok"] for r in core)

    @property
    def all_passed(self):
        return self.passed and all(r["ok"] for r in self.rows)

    def done(self, exit=True):
        core = [r for r in self.rows if not r["advanced"]]
        adv = [r for r in self.rows if r["advanced"]]
        color = GREEN if self.passed else RED
        line = f"== {self.name}: "
        if adv:
            line += f"핵심 {sum(r['ok'] for r in core)}/{len(core)} 통과 ({'PASS' if self.passed else 'FAIL'})"
            line += f" · 심화 {sum(r['ok'] for r in adv)}/{len(adv)}"
        else:
            line += f"{sum(r['ok'] for r in core)}/{len(core)} 통과 ({'PASS' if self.passed else 'FAIL'})"
        print(f"\n{color}{line}{END}")
        if self.out_dir:
            self.out_dir.mkdir(parents=True, exist_ok=True)
            with open(self.out_dir / "check_result.json", "w") as f:
                json.dump({"name": self.name, "passed": self.passed, "all_passed": self.all_passed,
                           "rows": self.rows, **self.extra}, f, indent=1, ensure_ascii=False, default=float)
        ok = self.all_passed if os.environ.get("DSP_STRICT") == "1" else self.passed
        if exit:
            sys.exit(0 if ok else 1)
        return ok


def section(title):
    """시험 묶음 사이에 빈 줄과 굵은 제목을 찍는다.

    Print a blank line and a bold heading between groups of checks.
    """
    print(f"\n{BOLD}{title}{END}")
