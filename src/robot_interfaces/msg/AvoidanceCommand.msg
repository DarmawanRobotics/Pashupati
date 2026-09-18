# Output of obstacle_avoidance_node, consumed by path_follower_node.

std_msgs/Header header
float32 steering_bias      # rad/s, added to the controller's angular cmd
float32 velocity_scale     # 0..1 multiplier on target linear velocity
bool emergency              # hard-stop, obstacle inside emergency cone/distance