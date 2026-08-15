# SOURCES — 실사 정사영상 후보 조사 및 최종 선정

두 단계 조사를 거쳤다: (1) 데이터셋/제공처 단위 후보 비교(스크래치 리서치), (2) 채택한 제공처
(PNOA-MA) 내에서 실제 구획 단위 후보 비교(이 태스크에서 직접 수행, FFT 실측 포함).

## 1단계 — 데이터셋/제공처 후보 (선행 리서치 요약)

조사 기준: 재배포 가능 라이선스 + GSD ≤25cm + 뚜렷한 열 구조(과수원) + 다운로드 가능 + 연속
≥2ha. 전체 조사 원문은 세션 스크래치의 `imagery_candidates.md`(2026-08-14 조사)에 있다.

| 제공처 | 지역/작물 | GSD | 라이선스 | 판정 |
|---|---|---|---|---|
| **PNOA Ortofoto Máxima Actualidad**(IGN/CNIG, 스페인) | 스페인 전역(레리다 세그리아 등 사과/배/복숭아 산지 포함) | 0.25m(연간본)~0.15m(최신본) | Orden FOM/2807/2015, CC BY 4.0(WMS 서비스 자체 명시 — 이 태스크에서 재확인) | **채택** |
| BD ORTHO®(IGN, 프랑스) | 프랑스 전역(사과 산지 다수) | 0.20m | Licence Ouverte/Open Licence 2.0(Etalab) | 차선(2위) — 정확한 과수원 좌표 미특정 |
| UAV RGB Pistachio Orchard Dataset(Zenodo 7239197/7271542) | 스페인, 피스타치오 2.03ha | 미기재 | CC-BY-4.0 | 차선(3위) — 작물·면적·형태 상이 |
| USDA NAIP(미국) | 워싱턴주 사과 산지 | 0.6m(표준) | 공공 도메인 | GSD 기준 미달 |
| 국토지리정보원(한국) | 경북 사과 산지 | 0.12~0.25m | 불명확(공공누리 유형 미명시, 국외반출 제한 이력) | 탈락(라이선스 근거 불충분) |
| MinneApple | 미국, 사과 개별사진 | - | CC BY-NC-SA 3.0 | 탈락(비상업 조건) |
| Fuji-SfM Dataset | 스페인, 사과(Fuji) | 미기재 | CC-BY-4.0 | 탈락(11그루 규모, 면적 기준 미달) |

## 2단계 — PNOA-MA 내 구획 후보 (이 태스크 실측)

레리다 세그리아 5개 지자체(Alcarràs, Torres de Segre, Aitona, Soses, Seròs)를 IGN 공식 WMS로
정찰(3km×3km, 1m/px) 후 유망 블록 4곳을 고해상도(0.25m/px)로 재확보하고, 2D FFT(excess-green
지수) 기반 자동 열 간격 측정을 적용했다. 측정 스크립트: `measure_rows.py`/`scan_rows.py`(세션
스크래치).

| 후보 | 위치(UTM31N 중심) | 면적(대략) | 실측 열 간격 | FFT 순도 | 비고 |
|---|---|---|---|---|---|
| A. Aitona | E288616.8 N4596084.9 | ~4.3ha | 5.19m | 247k | 농로가 대각선 관통, 하위 구획 수령 차이 |
| **C. Soses ⭐ 채택** | E290746.9 N4600493.4 | ~2.7ha(최종 크롭 기준) | 4.95m(크롭 실측, 큰 블록 기준 5.04m) | 474k(크롭 실측)·631k(큰 블록) — 4곳 중 최고 | 단일 균질 블록, 열 연속성 최상 |
| B. Torres de Segre | E293845.6 N4600698.3 | ~3.9ha | 4.67~4.80m | 330~420k | 기준(3~4.5m)에 가장 근접했으나 순도·연속성에서 C보다 낮음 |
| (참고) D. Seròs | E284307.4 N4593314.1 | ~3.3ha | 5.17m | 118k | 소구획 조각남, 백업만 |

## 최종 선정 — C. Soses

**컨트롤러 승인(2026-08-15)**: "단일 균질 블록(순도 631k 최고), 열 연속성 최상 — 조각난 B보다
시뮬 농장 기반으로 적합"이라는 근거로 B(Torres de Segre, 열 간격은 기준에 더 근접)가 아닌
C(Soses)를 채택했다.

**스펙 이탈 기록(findings 인계 항목)**: 실측 열 간격 4.95m(최종 크롭 기준)는 브리프 기준(3~4.5m
— `row_spacing=3.50m`인 시뮬 월드와의 시각적 정합을 노린 가이드라인)을 약 10~40% 초과한다. 4개
후보 전부 3.5~5.2m대로 나타나 이 특정 소지역(세그리아 하천변, Aitona/Seròs/Soses 인접)이
사과·배보다 열 간격이 넓은 복숭아/천도 위주 재배지일 가능성을 시사한다(항공사진만으로 수종 확정은
불가 — 미검증). **컨트롤러 판정**: "기하가 farm.json 데이터 주도라 운용 영향 없음"으로 이 이탈을
수용.

위치: Soses, Segrià, Lleida, Catalunya, España (WGS84 약 41.5314°N, 0.4809°E).

## 다운로드 경로 — CNIG 다운로드센터 → IGN WMS 대체 (컨트롤러 승인)

브리프는 `centrodedescargas.cnig.es`(CNIG 다운로드센터)에서 MTN25 도엽 COG를 내려받아 크롭하는
경로를 지시했다. 실제로는:
- CNIG 다운로드센터가 JS 렌더링 SPA라 WebFetch(정적 파싱)로 콘텐츠를 못 얻었고,
- 환경에 GDAL/rasterio가 없어(설치 금지) 대용량 COG 후처리가 불가능했다.

대신 **IGN 공식 WMS**(`https://www.ign.es/wms-inspire/pnoa-ma`)로 동일 PNOA-MA 데이터셋을
EPSG:25831 bbox 직접 지정으로 받았다. 컨트롤러 승인 사유: "동일 기관·동일 PNOA-MA 데이터셋이고
AccessConstraints에 CC BY 4.0이 명시돼 라이선스 근거가 더 명확하다." 상세 근거는
`LICENSE-DATA.md` 참조.

## 촬영일·해상도 — 소스 메타데이터로 검증

`OI.MosaicElement`(모자이크 레이어, `queryable=1`)에 대해 최종 크롭 중심 좌표로 `GetFeatureInfo`를
질의해 실제 촬영월과 해상도를 1차 소스에서 직접 확인했다(추정치 아님):

```
GET .../pnoa-ma?REQUEST=GetFeatureInfo&QUERY_LAYERS=OI.MosaicElement&...
{"properties": {"Fecha": "2024-04", "Resolucion": "0.25"}}
```

→ `orchard_ortho_meta.json`의 `acquired: "2024-04"`, `gsd_m: 0.25`는 이 응답값을 그대로 반영했다
(일 단위 정보는 소스에 없어 임의 보완하지 않음).
