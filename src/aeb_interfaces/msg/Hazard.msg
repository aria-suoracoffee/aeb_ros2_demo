# Nearest in-path obstacle as seen by the perception node.
std_msgs/Header header
float32 range          # m, longitudinal distance to the obstacle (inf if clear)
float32 closing_speed  # m/s, positive when the gap is shrinking
float32 ttc            # s, range / closing_speed (inf if not closing)
bool valid             # true when a real target is present
