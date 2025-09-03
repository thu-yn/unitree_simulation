uint8 version_h              # 版本高位
uint8 version_l              # 版本低位
uint8 bms_status             # BMS状态
uint8 SOC                    # 电量百分比 (0-100%)
int32 current                # 电流 (mA)
uint16 cycle                 # 循环次数
int8[8] BQ_NTC               # BQ芯片温度 (°C)
int8[8] MCU_NTC              # MCU温度 (°C)  
uint16[30] cell_vol          # 电池单体电压 (mV)