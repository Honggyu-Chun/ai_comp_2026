#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""competition_control_udp_bridge._normalize_steer self-check.

EgoCtrlCmd Steer CMD = -1 ~ 1, ±1 이 ±40deg 에 대응한다.
ROS 없이 실행:  python3 test_normalize_steer.py
"""
import math
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

# rospy / morai_msgs 없이 모듈을 import 하기 위한 최소 스텁.
sys.modules.setdefault("rospy", types.SimpleNamespace(
    get_param=lambda *a, **k: (a[1] if len(a) > 1 else None),
    init_node=lambda *a, **k: None, Subscriber=lambda *a, **k: None,
    Timer=lambda *a, **k: None, Duration=lambda *a, **k: None,
    loginfo=lambda *a, **k: None, logwarn_throttle=lambda *a, **k: None,
    on_shutdown=lambda *a, **k: None, spin=lambda: None,
    ROSInterruptException=Exception,
))
_morai = types.ModuleType("morai_msgs")
_morai.msg = types.SimpleNamespace(CtrlCmd=object)
sys.modules.setdefault("morai_msgs", _morai)
sys.modules.setdefault("morai_msgs.msg", _morai.msg)

from competition_control_udp_bridge import MAX_STEER_RAD, _normalize_steer


def demo():
    # 중립
    assert _normalize_steer(0.0) == 0.0

    # 최대 조향각 -> 정확히 ±1
    assert abs(_normalize_steer(math.radians(40.0)) - 1.0) < 1e-12
    assert abs(_normalize_steer(math.radians(-40.0)) + 1.0) < 1e-12

    # 범위 밖은 잘려서 ±1 을 넘지 않는다
    assert _normalize_steer(math.radians(60.0)) == 1.0
    assert _normalize_steer(math.radians(-60.0)) == -1.0
    assert _normalize_steer(100.0) == 1.0

    # 선형: 20deg -> 0.5, 10deg -> 0.25
    assert abs(_normalize_steer(math.radians(20.0)) - 0.5) < 1e-12
    assert abs(_normalize_steer(math.radians(10.0)) - 0.25) < 1e-12

    # 부호 대칭
    for deg in (5.0, 17.3, 34.0, 40.0):
        r = math.radians(deg)
        assert abs(_normalize_steer(r) + _normalize_steer(-r)) < 1e-12

    # 어떤 입력에도 프로토콜 범위를 벗어나지 않는다
    for deg in range(-180, 181, 7):
        assert -1.0 <= _normalize_steer(math.radians(deg)) <= 1.0

    # LQR 이 실제로 내보내는 상한 34deg
    assert abs(_normalize_steer(0.5934119) - 34.0 / 40.0) < 1e-4

    print("normalize_steer OK (MAX_STEER_RAD=%.6f rad = %.1f deg, 34deg -> %.4f)"
          % (MAX_STEER_RAD, math.degrees(MAX_STEER_RAD), _normalize_steer(0.5934119)))


if __name__ == "__main__":
    demo()
