uint8 mode           # 电机当前模式
float32 q            # 电机当前位置 (rad)
float32 dq           # 电机当前速度 (rad/s)
float32 ddq          # 电机当前加速度 (rad/s²)
float32 tauEst       # 当前估计输出力矩 (N*m)
float32 q_raw        # 电机原始位置
float32 dq_raw       # 电机原始速度
float32 ddq_raw      # 电机原始加速度
int8 temperature     # 电机温度（有延迟）
uint32[2] reserve    # 保留字段