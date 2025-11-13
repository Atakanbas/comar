# Obstacle avoidance commands for autonomous ground rover
# @author Atkan
# @date 2024

uint64 timestamp		# time since system start (microseconds)

bool avoidance_active		# true if avoidance is active
float32 avoidance_direction	# -1.0 to 1.0 (left to right)
float32 avoidance_speed		# 0.0 to 1.0 (slow to normal)
bool obstacle_detected		# true if obstacle is detected
