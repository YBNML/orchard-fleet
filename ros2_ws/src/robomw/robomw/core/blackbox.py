"""
블랙박스 — 궤적·이벤트 링 npz 덤프

순수 파이썬 + numpy. ROS 무관.

feed_pose 는 1 Hz 주기(control_agent 의 텔레메트리 타이머)를 가정한다.
최대 900초 윈도우로 posture 와 event 를 저장했다가 dump() 로 npz 파일로 낸다.
"""
from __future__ import annotations

import json
import time
from collections import deque

import numpy as np


class Blackbox:
    """궤적·이벤트 링 데이터 저장소"""

    def __init__(self, maxlen_s: float = 900):
        """
        Args:
            maxlen_s: 최대 기억 시간(초). 상한 900초.
        """
        self.maxlen_s = min(float(maxlen_s), 900.0)
        # 1 Hz 주기를 가정하면 maxlen_s 초에 maxlen_s 개 샘플
        # 실제로는 더 많을 수 있지만(고주기) deque 크기로 수량을 제한한다
        self._max_poses = int(self.maxlen_s) + 1
        self._poses = deque(maxlen=self._max_poses)  # [(t, x, y, yaw), ...]
        self._events = deque(maxlen=50)  # 50개 링 — 독립적인 이벤트 저장소

    def effective_window(self, window_s: float) -> float:
        """요청 윈도우를 900초 상한에 맞춘다."""
        return min(float(window_s), 900.0)

    def feed_pose(self, t: float, x: float, y: float, yaw: float) -> None:
        """포즈를 저장한다. 1 Hz 주기를 가정한다.

        Args:
            t: 타임스탬프(초)
            x: x 위치(미터)
            y: y 위치(미터)
            yaw: 요(라디안)
        """
        self._poses.append((float(t), float(x), float(y), float(yaw)))

    def feed_event(self, event: dict) -> None:
        """이벤트를 저장한다. 50개 링 버퍼.

        Args:
            event: 이벤트 사전 (kind, t, ... 등)
        """
        self._events.append(dict(event))

    def dump(self, path: str, window_s: float = 900) -> dict:
        """궤적·이벤트를 npz 파일로 저장하고 메타데이터를 반환한다.

        Args:
            path: 저장할 npz 파일 경로
            window_s: 저장할 시간 윈도우(초). 900초 상한 적용됨.

        Returns:
            dict with keys:
                "path": 저장 경로
                "bytes": 파일 크기(바이트)
                "events": 저장된 이벤트 수
                "poses": 저장된 포즈 수
        """
        window_s = self.effective_window(window_s)

        # 포즈 필터링: 시간 윈도우로 자르기
        if not self._poses:
            poses_array = np.empty((0, 4), dtype=np.float32)
            events_array = np.array([], dtype=object)
        else:
            poses_list = list(self._poses)
            if poses_list:
                max_t = poses_list[-1][0]  # 가장 최근 타임스탬프
                cutoff_t = max_t - window_s
                # cutoff_t 이후의 포즈만 남긴다
                filtered_poses = [p for p in poses_list if p[0] > cutoff_t]
                if filtered_poses:
                    poses_array = np.array(filtered_poses, dtype=np.float32)
                else:
                    poses_array = np.empty((0, 4), dtype=np.float32)
            else:
                poses_array = np.empty((0, 4), dtype=np.float32)

            # 이벤트도 같은 윈도우로 필터링
            if self._events:
                events_list = list(self._events)
                filtered_events = [e for e in events_list if e.get("t", 0.0) > cutoff_t]
                # JSON 문자열 배열로 저장
                events_array = np.array(
                    [json.dumps(e) for e in filtered_events], dtype=object
                )
            else:
                events_array = np.array([], dtype=object)

        # npz 파일로 저장
        np.savez(path, poses=poses_array, events=events_array)

        # 파일 크기 확인
        import os
        file_size = os.path.getsize(path)

        return {
            "path": path,
            "bytes": file_size,
            "events": len(events_array),
            "poses": len(poses_array),
        }
