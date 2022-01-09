####################################################
# Event constants
####################################################
uint8 TWELITE_EVENT_UNKNOWN     = 0
uint8 TWELITE_EVENT_DICE_1      = 1
uint8 TWELITE_EVENT_DICE_2      = 2
uint8 TWELITE_EVENT_DICE_3      = 3
uint8 TWELITE_EVENT_DICE_4      = 4
uint8 TWELITE_EVENT_DICE_5      = 5
uint8 TWELITE_EVENT_DICE_6      = 6
uint8 TWELITE_EVENT_DICE_SHAKE  = 8
uint8 TWELITE_EVENT_DICE_MOVE   = 16
####################################################
# Magnet constants
####################################################
uint8 TWELITE_MAGNET_OPEN       = 0
uint8 TWELITE_MAGNET_CLOSE_N    = 1
uint8 TWELITE_MAGNET_CLOSE_S    = 2
uint8 TWELITE_MAGNET_UNKNOWN    = 3
####################################################
# Event constants
####################################################
uint8 TWELITE_PAL_OPEN_CLOSE_PAL        = 1
uint8 TWELITE_PAL_AMBIENT_PAL           = 2
uint8 TWELITE_PAL_MOTION_PAL            = 3
uint8 TWELITE_PAL_NOTICE_PAL            = 4
uint8 TWELITE_PAL_TWELITE_CUE           = 5
uint8 TWELITE_PAL_TWELITE_ARIA          = 6
####################################################
# Sensor Name
####################################################
uint8 TWELITE_SENSOR_ANALOG          = 	16
uint8 TWELITE_SENSOR_LM61            = 	17
uint8 TWELITE_SENSOR_SHT21           = 	49
uint8 TWELITE_SENSOR_ADT7410         = 	50
uint8 TWELITE_SENSOR_MPL115A2        = 	51
uint8 TWELITE_SENSOR_LIS3DH          = 	52
uint8 TWELITE_SENSOR_ADXL34X         = 	53
uint8 TWELITE_SENSOR_TSL2561         = 	54
uint8 TWELITE_SENSOR_L3GD20          = 	55
uint8 TWELITE_SENSOR_S11059_02DT     = 	56
uint8 TWELITE_SENSOR_BME280          = 	57
uint8 TWELITE_SENSOR_SHT31           = 	58
uint8 TWELITE_SENSOR_SHTC3           = 	59
uint8 TWELITE_SENSOR_ADXL362         = 	98
uint8 TWELITE_SENSOR_PAL             = 	128
uint8 TWELITE_SENSOR_MULTISENSOR     = 	209
uint8 TWELITE_SENSOR_BUTTON          = 	254

####################################################
# Data
####################################################
std_msgs/Header     header

std_msgs/UInt32         logical_id
std_msgs/UInt32         end_device_sid
std_msgs/UInt32         router_sid
std_msgs/UInt32         version
std_msgs/UInt32         pal
std_msgs/UInt8          sensor

std_msgs/Float32        lqi

std_msgs/UInt64         event_id
std_msgs/UInt8          magnet
std_msgs/UInt32         power
std_msgs/UInt32         adc

std_msgs/UInt32         accel_sampling
geometry_msgs/Accel     accel_x
geometry_msgs/Accel     accel_y
geometry_msgs/Accel     accel_z
