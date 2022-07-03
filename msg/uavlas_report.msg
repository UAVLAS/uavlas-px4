# UAVLAS_REPORT message data

uint64 timestamp		# time since system start (microseconds)

uint16 id			# Beacon ID
uint16 status			# Sensor status

# Sensor return relative data of beacon offset

float32 pos_x # Relative Position of receiver in target frame X axis (in m)
float32 pos_y # Relative Position of receiver in target frame Y axis (in m)
float32 pos_z # Relative Position of receiver in target frame Z axis (in m)

float32 vel_x # Ralative velocity of  receiver in target frame. X axis (m/s)
float32 vel_y # Ralative velocity of  receiver in target frame. Y axis (m/s)
float32 vel_z # Ralative velocity of  receiver in target frame. Z axis (m/s)

float32 gimu_roll  # orientation of ground unit - roll (deg)
float32 gimu_pitch # orientation of ground unit - pitch (deg)
float32 gimu_yaw   # orientation of ground unit - yaw (compass) (deg)

float32 mrxyaw	#boady relative orientation in MultiRX mode (deg)

float32 sq    # signal quality %
float32 ss    # common lighting level (db)
float32 sl    # signal level (%)

 # Estimated Position Data
float32  rel_pos_ned_n # Relative Position of target in NED (North)[m]
float32  rel_pos_ned_e # Relative Position of target in NED (East)[m]
float32  rel_pos_ned_d # Relative Position of target in NED (Down)[m]

float32  rel_vel_ned_n # Ralative velocity of target in NED (North) [m/s]
float32  rel_vel_ned_e # Ralative velocity of target in NED (East) [m/s]
float32  rel_vel_ned_d # Ralative velocity of target in NED (Down) [m/s]

float32  abs_pos_ned_n # Absolute Position of target in NED (North)[m]
float32  abs_pos_ned_e # Absolute Position of target in NED (East)[m]
float32  abs_pos_ned_d # Absolute Position of target in NED (Down)[m]

float32  abs_vel_ned_n # Absolute velocity of target in NED (North) [m/s]
float32  abs_vel_ned_e # Absolute velocity of target in NED (East) [m/s]
float32  abs_vel_ned_d # Absolute velocity of target in NED (Down) [m/s]

bool  receiver_id_valid # Get receiver signal with valid ID
bool  abs_ned_valid # Absolute position and velocity of target in NED valid
# Absolute position and velocity of target in NED calculated by prediction algirithm
bool  abs_ned_prediction_active



