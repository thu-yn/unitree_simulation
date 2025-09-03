uint8 mode           # 电机目标模式
float32 q            # 电机目标位置 (rad)
float32 dq           # 电机目标速度 (rad/s)
float32 tau          # 电机目标力矩 (N*m)
float32 Kp           # 电机位置刚度系数
float32 Kd           # 电机阻尼系数
uint32[3] reserve    # 保留字段