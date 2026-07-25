# Livox 커스텀 메시지 (livox_ros_driver2 원본과 필드·순서 동일)
std_msgs/Header header
uint64 timebase         # 프레임 첫 점의 절대 시각 [ns]
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
