# Output of the AEB controller, for logging / HMI / diagnostics.
std_msgs/Header header

uint8 CLEAR=0
uint8 WARN=1
uint8 BRAKE=2

uint8 state
string state_label
float32 ttc             # s
float32 required_decel  # m/s^2 needed to stop short of the obstacle
bool braking            # true while a brake command is being issued
