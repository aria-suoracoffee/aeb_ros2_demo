# One raw radar detection for the in-path target (or a miss).
std_msgs/Header header
bool detected
float32 range        # m
float32 range_rate   # m/s, raw radial velocity (negative = approaching)
