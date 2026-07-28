uint8 level          # 电量 0~100%
uint16 pt_status     # 保护状态位
float32 power        # 功率，正为放电，负为充电，单位：W
float32 voltage      # 电压，单位：V
int8 temperature     # 温度, 单位：C

# pt_status 位定义
uint16 CELL_OVER_VOLTAGE = 1        # bit0  单体过压保护
uint16 CELL_UNDER_VOLTAGE = 2       # bit1  单体欠压保护
uint16 PACK_OVER_VOLTAGE = 4        # bit2  整组过压保护
uint16 PACK_UNDER_VOLTAGE = 8       # bit3  整组欠压保护
uint16 CHARGE_OVER_TEMP = 16        # bit4  充电过温保护
uint16 CHARGE_UNDER_TEMP = 32       # bit5  充电低温保护
uint16 DISCHARGE_OVER_TEMP = 64     # bit6  放电过温保护
uint16 DISCHARGE_UNDER_TEMP = 128   # bit7  放电低温保护
uint16 CHARGE_OVER_CURRENT = 256    # bit8  充电过流保护
uint16 DISCHARGE_OVER_CURRENT = 512 # bit9  放电过流保护
uint16 SHORT_CIRCUIT = 1024         # bit10 短路保护
uint16 FRONTEND_IC_ERROR = 2048     # bit11 前端检测IC错误
uint16 SOFTWARE_LOCK_MOS = 4096     # bit12 软件锁定MOS
# bit13~bit15 预留
