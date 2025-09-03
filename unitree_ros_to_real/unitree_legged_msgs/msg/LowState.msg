uint8[2] head                # 消息头
uint8 levelFlag              # 控制级别标志
uint8 frameReserve           # 帧保留

uint32[2] SN                 # 序列号
uint32[2] version            # 版本信息
uint16 bandWidth             # 带宽设置

IMU imu                      # 惯性测量单元数据
MotorState[20] motorState    # 20个电机状态
BmsState bms                 # 电池管理状态
int16[4] footForce           # 足端力传感器数据
int16[4] footForceEst        # 足端力估计
uint32 tick                  # 时间戳
uint8[40] wirelessRemote     # 无线遥控数据
uint32 reserve               # 保留
uint32 crc                   # 校验码

# 以下为旧版本兼容字段（Aliengo不包含）
Cartesian[4] eeForceRaw      # 末端执行器原始力
Cartesian[4] eeForce         # 末端执行器力（实际为1DOF，但用3DOF显示）
Cartesian position           # 位置（将删除）
Cartesian velocity           # 速度（将删除）
Cartesian velocity_w         # 角速度（将删除）