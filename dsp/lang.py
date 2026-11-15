"""노트북에 보이는 글자(출력, 그래프 제목·범례, 표, 오류 안내)의 언어. 기본은 한국어.

영어 노트북(lesson_en.ipynb)은 첫 셀에서 `set_language("en")`을 부른다. 환경 변수 ICARUS_LANG=en 으로도 바꿀 수 있다.
글자를 쓰는 곳에서는 tr("한국어", "English")처럼 두 말을 같이 적는다.

Language of the text the notebooks show (prints, plot titles and legends, tables, error hints).
Korean by default; English after `set_language("en")` or with ICARUS_LANG=en.
"""
from __future__ import annotations

import os

_STATE = {"lang": "en" if os.environ.get("ICARUS_LANG", "").lower().startswith("en") else "ko"}


def set_language(lang):
    """"ko"(한국어) 또는 "en"(English)."""
    lang = str(lang).lower()[:2]
    if lang not in ("ko", "en"):
        raise ValueError('set_language("ko") 또는 set_language("en")')
    _STATE["lang"] = lang


def get_language():
    return _STATE["lang"]


def tr(ko, en):
    """지금 언어에 맞는 쪽을 돌려준다."""
    return en if _STATE["lang"] == "en" else ko
