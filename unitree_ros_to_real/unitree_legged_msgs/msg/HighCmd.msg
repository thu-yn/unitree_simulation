uint8[2] head                # 消息头
uint8 levelFlag              # 控制级别标志
uint8 frameReserve           # 帧保留

uint32[2] SN                 # 序列号
uint32[2] version            # 版本信息
uint16 bandWidth             # 带宽设置

uint8 mode                   # 控制模式
uint8 gaitType               # 步态类型
uint8 speedLevel             # 速度级别
float32 footRaiseHeight      # 足部抬起高度
float32 bodyHeight           # 身体高度
float32[2] position          # 位置 [x, y]
float32[3] euler             # 欧拉角 [roll, pitch, yaw]
float32[2] velocity          # 速度 [vx, vy]
float32 yawSpeed             # 偏航角速度
float32[2] dComXy            # 质心偏移
float32[2] dstandFootXy      # 站立足位偏移

BmsCmd bms                   # 电池管理命令
LED[4] led                   # LED控制（4个LED）
uint8[40] wirelessRemote     # 无线遥控数据
uint32 reserve               # 保留
uint32 crc                   # 校验码