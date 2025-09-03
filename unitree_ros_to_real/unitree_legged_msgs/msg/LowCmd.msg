uint8[2] head                # 消息头
uint8 levelFlag              # 控制级别标志
uint8 frameReserve           # 帧保留

uint32[2] SN                 # 序列号
uint32[2] version            # 版本信息
uint16 bandWidth             # 带宽设置

MotorCmd[20] motorCmd        # 20个电机控制命令
BmsCmd bms                   # 电池管理命令
uint8[40] wirelessRemote     # 无线遥控数据
uint32 reserve               # 保留
uint32 crc                   # 校验码