# findings 원자료

`docs/findings/*.md` 가 인용하는 수치의 근거다. 문서의 주장은 여기서 다시 셀 수
있어야 한다 — 원자료 없이 남은 결론은 나중에 재판정이 불가능하다(2026-08-13
T8 이 근접 노출 A/B 대조를 사후 재판정하지 못한 이유가 그것이었다).

## 2026-08-15 — 실사 월드 최종 게이트 (`2026-08-15-photoreal-world.md`)

| 파일 | 내용 | 어느 주장의 근거인가 |
|---|---|---|
| `2026-08-15-gate-track-scout01-1hz.csv` | scout01 전 구간 참값 대조, 1 Hz 다운샘플(원본 10 Hz 37,300행) | RTF(Δt_sim/Δt_wall) · 통로 내 횡/종오차 · 주행거리 (§4, §9.0) |
| `2026-08-15-gate-track-scout02-1hz.csv` | scout02 같은 것 | 같음 + 재개 후 종오차 누적 (§9.1) |
| `2026-08-15-slip-scout02-alley16.csv` | 통로 16 정지 사건 전후 90/60초, **10 Hz 전량** | 추정 점프의 크기·방향·요 동시 점프 (§9.1) |
| `2026-08-15-slip-scout01-alley7.csv` | 통로 7 정지 사건 같은 발췌 | 같음(두 번째 재현) |
| `2026-08-15-slip-scout01-alley1.csv` | 자력회복 건 같은 발췌 | 재시도가 먹힌 경우와의 대조 (§9.1) |
| `2026-08-15-slip-localizer-log.txt` | `map_localizer` 상태·경보 로그 세 구간 | `scan_travel` 의 (travel, conf) 실측값 · 구조점 감소 (§9.1) |

### 열 규약 (CSV 공통)

```
t_wall_s   참값 기록 시작 기준 벽시계 초
t_sim_s    시뮬레이션 시각 (RTF = Δt_sim / Δt_wall)
gt_*       gz 참값 (/<robot>/gz_ground_truth)
est_*      추정 (map → <robot>/base_link TF)
err_*      est − gt
```

기록 도구는 `scripts/39_verify_localization_live.py` 의 로직을 CSV 로 흘려 쓰는
스크래치 판이다(39 는 SIGINT 시 표본을 잃는 알려진 버그가 있다 — T3 이연 항목).

### 로그 읽는 법

```
슬립 감지 — 오도 A m vs 스캔 B m (상관 C) · 자세를 앵커에 고정
```

- `A` = 오도메트리 변위, `B` = `rowlocalize.scan_travel()` 이 낸 전진 변위,
  `C` = 그 상관계수(문턱 0.3, `map_localizer._check_slip`).
- **`B ≈ 0` 인데 `C` 가 높은 것이 상관 축퇴**다 — 감시기가 "확신을 갖고
  안 갔다"고 답한다.
- 상태 줄의 `구조점` 이 150 밑으로 내려가면 `_slip_points` 가 줄기 구조점 대신
  **생구름 폴백**으로 넘어간다. 세 사건 모두 그 직전 구간이다.
