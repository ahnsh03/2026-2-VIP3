# 수동 주차면 형식

제공 GT는 [data/katri/](../data/katri/README.md)에 있다. 이 형식은 제공본이 없을 때 로더가 읽는 **수동 대안**이며 센서 인지 실패를 숨기는 런타임 fallback이 아니다.

최상위 배열의 각 항목에 고유 `idx`와 polygon `points`를 둔다. 좌표는 사용하려는 맵과 정합한 로컬 미터 좌표이며, 자료별 원점과 변환 근거를 별도 기록한다. 과거 KATRI 원점을 모든 맵에 고정 적용하지 않는다.

```json
[
  {
    "idx": "P0001",
    "points": [[0, 0, 0], [5, 0, 0], [5, 2.5, 0], [0, 2.5, 0]],
    "center_point": [2.5, 1.25, 0],
    "width": 2.5,
    "length": 5.0,
    "angle": 90.0,
    "parking_direction": null
  }
]
```

위는 형식 예시이며 시뮬 목표가 아니다. `angle`을 차량 heading으로 해석하지 않는다. `points`로 형상을 계산하고 입구·진입 방향을 직접 확인한다. 차량을 세운 pose에 차량 치수를 더해 얻은 사각형은 주차칸 경계가 아니다.

실제 로더의 필드 처리 기준은 `mgeo_layers.normalise_parking_space`, 좌표·후륜축 목표 기준은 [데이터 문서](../../../docs/system/data.md)와 [인터페이스](../../../docs/system/design.md)를 따른다.
