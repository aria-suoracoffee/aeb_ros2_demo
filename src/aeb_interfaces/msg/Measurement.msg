# A per-sensor estimate of the in-path target, consumed by the fusion node.
# Each sensor reports only the quantities it actually measures, each with its
# own variance, so the filter can weight them correctly.
std_msgs/Header header
string source           # "lidar" | "radar"
bool valid              # false => this sensor has no target this cycle

bool has_range
float32 range           # m
float32 range_var       # m^2

bool has_rate
float32 closing_speed   # m/s, positive = approaching
float32 rate_var        # (m/s)^2
