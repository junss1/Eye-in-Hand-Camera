

# speak.py v0.100 2026-02-06
# [이번 버전에서 수정된 사항]
# - (기능구현) edge-tts(ko-KR-HyunsuMultilingualNeural)로 TTS 출력하도록 speak() 구현
# - (기능구현) edge-tts 바이너리 자동 탐색(shutil.which 또는 ~/venvs/edge_tts/bin/edge-tts)
# - (유지) 호출부(AuthServer 등)에서 speak(text) 인터페이스는 그대로 유지

"""
TTS helper: Microsoft edge-tts 기반 음성 출력

요구사항
- edge-tts 설치(venv 또는 시스템)
- ffplay(권장) 또는 다른 플레이어 필요

기본 동작
- text -> edge-tts로 mp3 생성 -> ffplay로 재생
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from typing import Optional

# 기본 보이스: 현수(멀티링구얼)
DEFAULT_VOICE = "ko-KR-HyunsuMultilingualNeural"

# venv 기본 경로(네가 만든 구조 기준)
DEFAULT_EDGE_TTS_BIN = os.path.expanduser("~/venvs/edge_tts/bin/edge-tts")


def _find_edge_tts_bin() -> str:
    # 1) 환경변수로 지정 가능
    env_bin = os.environ.get("EDGE_TTS_BIN", "").strip()
    if env_bin and os.path.isfile(env_bin) and os.access(env_bin, os.X_OK):
        return env_bin

    # 2) PATH에서 찾기
    p = shutil.which("edge-tts")
    if p:
        return p

    # 3) 네 venv 기본 위치
    if os.path.isfile(DEFAULT_EDGE_TTS_BIN) and os.access(DEFAULT_EDGE_TTS_BIN, os.X_OK):
        return DEFAULT_EDGE_TTS_BIN

    raise FileNotFoundError(
        "edge-tts 바이너리를 찾을 수 없음. "
        "1) venv 활성화 후 실행하거나, "
        "2) launch에서 PATH에 ~/venvs/edge_tts/bin 추가하거나, "
        "3) EDGE_TTS_BIN 환경변수로 경로 지정하세요."
    )


def _find_player() -> Optional[list]:
    # ffplay가 mp3 재생 가장 쉬움
    ffplay = shutil.which("ffplay")
    if ffplay:
        return [ffplay, "-nodisp", "-autoexit", "-loglevel", "quiet"]

    # mpv 있으면 이것도 가능
    mpv = shutil.which("mpv")
    if mpv:
        return [mpv, "--no-video", "--really-quiet"]

    # aplay는 mp3 직접 재생이 안됨(보통 wav만)
    return None


def speak(
    text: str,
    *,
    voice: str = DEFAULT_VOICE,
    rate: str = "+0%",
    volume: str = "+0%",
    pitch: str = "+0Hz",
) -> None:
    """
    text: 읽을 문장
    voice: edge-tts 보이스명
    rate/volume/pitch: edge-tts 옵션 문자열(예: '+10%', '-5%', '+2Hz')
    """
    t = (text or "").strip()
    if not t:
        return

    edge_tts_bin = _find_edge_tts_bin()
    player = _find_player()

    # 출력 mp3 경로
    fd, out_mp3 = tempfile.mkstemp(prefix="tts_", suffix=".mp3")
    os.close(fd)

    try:
        # ✅ 중요: edge-tts는 파이프 stdin으로 텍스트 받는 방식이 아니라
        # --text(-t) 또는 --file(-f)을 써야 함.
        cmd = [
            edge_tts_bin,
            "--text",
            t,
            "--voice",
            voice,
            "--rate",
            rate,
            "--volume",
            volume,
            "--pitch",
            pitch,
            "--write-media",
            out_mp3,
        ]

        subprocess.run(cmd, check=True)

        if player is None:
            # 플레이어가 없으면 파일만 만들어지고 끝(조용히 종료)
            return

        subprocess.run(player + [out_mp3], check=False)

    finally:
        try:
            os.remove(out_mp3)
        except Exception:
            pass

