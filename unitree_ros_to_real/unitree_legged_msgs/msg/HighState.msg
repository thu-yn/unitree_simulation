uint8[2] head                   # 消息头
uint8 levelFlag                 # 控制级别标志
uint8 frameReserve              # 帧保留

uint32[2] SN                    # 序列号
uint32[2] version               # 版本信息  
uint16 bandWidth                # 带宽设置

IMU imu                         # 惯性测量单元
MotorState[20] motorState       # 20个电机状态
BmsState bms                    # 电池管理状态
int16[4] footForce              # 足端力
int16[4] footForceEst           # 足端力估计

uint8 mode                      # 当前模式
float32 progress                # 执行进度
uint8 gaitType                  # 当前步态类型
float32 footRaiseHeight         # 当前足部抬起高度
float32[3] position             # 当前位置 [x, y, z]
float32 bodyHeight              # 当前身体高度
float32[3] velocity             # 当前速度 [vx, vy, vz]
float32 yawSpeed                # 当前偏航角速度
float32[4] rangeObstacle        # 距离障碍物范围
Cartesian[4] footPosition2Body  # 足部相对身体位置
Cartesian[4] footSpeed2Body         # 足部相对身体速度

uint8[40] wirelessRemote        # 无线遥控数据
uint32 reserve                  # 保留
uint32 crc                      # 校验码