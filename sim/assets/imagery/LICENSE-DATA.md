# 데이터 라이선스 — `orchard_ortho.jpg`

## 원문·근거

`orchard_ortho.jpg`는 스페인 국립지리원(IGN, Instituto Geográfico Nacional) / CNIG(Centro
Nacional de Información Geográfica)가 운영하는 **PNOA(Plan Nacional de Ortofotografía Aérea)
Máxima Actualidad(PNOA-MA)** 정사영상 데이터셋에서, IGN 공식 WMS(`ign.es/wms-inspire/pnoa-ma`)를
통해 직접 발급받았다.

**1차 근거 — WMS 서비스 자체의 라이선스 명시** (2026-08-15, `GetCapabilities` 실조회로 확인):

```
GET https://www.ign.es/wms-inspire/pnoa-ma?SERVICE=WMS&REQUEST=GetCapabilities&VERSION=1.3.0

<Service>
  ...
  <Fees>No se aplican condiciones</Fees>
  <AccessConstraints>CC BY 4.0 scne.es</AccessConstraints>
</Service>
```

즉 서비스가 제공하는 데이터(레이어 `OI.OrthoimageCoverage` — "Cobertura ráster opaca de imágenes
de satélite y ortofotos PNOA de máxima actualidad" 포함)는 **CC BY 4.0**로 명시적으로 태깅돼 있다.
`scne.es`는 Sistema Cartográfico Nacional de España(스페인 국가 지도체계) 포털이다.

**2차 근거(보강) — 법령**: PNOA 데이터 전반의 재이용 조건은 `Orden FOM/2807/2015`
(BOE-A-2015-14129, 스페인 관보)에 근거하며, 무료 재이용을 허용하고 출처 표기 의무를 부과한다 —
위 WMS 서비스 단의 CC BY 4.0 명시와 실질적으로 동일한 조건이다.

## 의무 이행 — 출처 표기(Attribution)

CC BY 4.0은 출처 표기(attribution)를 요구한다. 이 이미지를 사용하는 모든 산출물(시뮬레이션 월드,
스크린샷, 대시보드 배경 등)에는 아래 표기를 포함해야 한다:

> Orthophoto: PNOA (Plan Nacional de Ortofotografía Aérea), © Instituto Geográfico Nacional
> (IGN) de España / CNIG, licencia CC BY 4.0. Fuente:
> https://www.ign.es/wms-inspire/pnoa-ma

간단 표기가 필요한 경우(예: 화면 하단 워터마크) — 코드 상수 `ORTHO_ATTRIBUTION`
(`server/fleet_server/api/farm_routes.py`)과 동일한 문구를 쓴다:

> PNOA cedido por © Instituto Geográfico Nacional · CC BY 4.0

※ 대시보드 렌더 문구와 동일하게 유지할 것.

## 원본 데이터 조회 경로 (재현용)

- WMS 엔드포인트: `https://www.ign.es/wms-inspire/pnoa-ma`
- 레이어: `OI.OrthoimageCoverage` (표시), `OI.MosaicElement`(메타데이터 조회 — 촬영월·해상도)
- `orchard_ortho.jpg`를 발급한 정확한 요청은 `orchard_ortho_meta.json`의
  `wms_request_url`/`wms_request_bbox_epsg25831` 참조.
- CC BY 4.0 전문: https://creativecommons.org/licenses/by/4.0/legalcode

## 참고 — 다운로드 경로 변경 이력

브리프 원안은 CNIG 다운로드센터(`centrodedescargas.cnig.es`)에서 MTN25 도엽 단위 COG 파일을
내려받는 경로를 지시했으나, (a) 해당 사이트가 JS 렌더링 SPA라 정적 도구로 접근 불가했고 (b) 이
환경에 GDAL/rasterio가 없어 COG 후처리가 불가능해, 동일 기관·동일 PNOA-MA 데이터셋을 제공하는
IGN 공식 WMS로 대체했다(컨트롤러 승인 완료, 2026-08-15). 데이터의 출처·라이선스 근거는 동일하다.
