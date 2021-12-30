# Event constants
uint8 TWELITE_EVENT_UNKNOWN             = 0
uint8 TWELITE_EVENT_DICE_1              = 1
uint8 TWELITE_EVENT_DICE_2              = 2
uint8 TWELITE_EVENT_DICE_3              = 3
uint8 TWELITE_EVENT_DICE_4              = 4
uint8 TWELITE_EVENT_DICE_5              = 5
uint8 TWELITE_EVENT_DICE_6              = 6
uint8 TWELITE_EVENT_DICE_SHAKE          = 7
uint8 TWELITE_EVENT_DICE_ACCELERATION   = 8
# Event constants
uint8 TWELITE_HALLIC_UNKNOWN    = 0
uint8 TWELITE_HALLIC_OPEN       = 1
uint8 TWELITE_HALLIC_CLOSE_N    = 2
uint8 TWELITE_HALLIC_CLOSE_S    = 3

# Data
std_msgs/Header     Header
std_msgs/UInt64         SequenceNumber
std_msgs/UInt32         Sensor
std_msgs/UInt32         LogicalID
std_msgs/UInt32         EndDeviceSID
std_msgs/UInt32         RouterSID
std_msgs/Float64        LQI
std_msgs/UInt64         EventID
std_msgs/UInt64         PALVersion
std_msgs/UInt8          Hallic

geometry_msgs/Accel Accel_X
geometry_msgs/Accel Accel_Y
geometry_msgs/Accel Accel_Z

