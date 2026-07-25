# Livox 포인트 (livox_ros_driver2 원본과 필드·순서 동일)
uint32 offset_time      # timebase 기준 상대 시각 [ns]
float32 x               # [m]
float32 y
float32 z
uint8 reflectivity      # 0~255  ※ PointCloud2 의 float32 intensity 와 타입이 다르다
uint8 tag
uint8 line              # MID-70 은 단일 레이저라 항상 0
