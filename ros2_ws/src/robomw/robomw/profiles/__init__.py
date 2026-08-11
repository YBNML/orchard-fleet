"""현장 프로파일 — 특정 작업 현장에서만 뜻이 있는 거동을 모은다.

기능(features/)과 나눈 기준: 다른 기체에 올려도 말이 되면 features, 현장의
기하·작업 규칙에 매여 있으면 profiles 다. 과수원 보스트로피돈 임무는 통로
격자·선회 구간·둑 경사를 전제로 하므로 profiles/orchard 에 있다.

적재는 완전 경로로 한다 (레지스트리는 점이 들어간 이름을 완전 경로로 본다):

    features: ["telemetry_state", "robomw.profiles.orchard.mission"]
"""
