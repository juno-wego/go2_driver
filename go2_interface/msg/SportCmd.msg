# SportCmd.msg
# High-level sport mode command for Unitree GO2
# Maps to SportClient API IDs from unitree_sdk2_python

# Command string — one of:
#   "damp"           → Damp (1001)
#   "balance_stand"  → BalanceStand (1002)
#   "stop_move"      → StopMove (1003)
#   "stand_up"       → StandUp (1004)
#   "stand_down"     → StandDown (1005)
#   "recovery_stand" → RecoveryStand (1006)
#   "sit"            → Sit (1009)
#   "rise_sit"       → RiseSit (1010)
#   "hello"          → Hello (1016)
#   "stretch"        → Stretch (1017)
#   "dance1"         → Dance1 (1022)
#   "dance2"         → Dance2 (1023)
#   "hand_stand"     → HandStand (2044), uses flag
#   "free_walk"      → FreeWalk (2045)
#   "free_bound"     → FreeBound (2046), uses flag
#   "free_jump"      → FreeJump (2047), uses flag
#   "free_avoid"     → FreeAvoid (2048), uses flag
#   "walk_upright"   → WalkUpright (2050), uses flag
#   "cross_step"     → CrossStep (2051), uses flag
#   "left_flip"      → LeftFlip (2041)
#   "back_flip"      → BackFlip (2043)
#   "front_flip"     → FrontFlip (1030)
#   "front_jump"     → FrontJump (1031)
#   "front_pounce"   → FrontPounce (1032)
#   "scrape"         → Scrape (1029)
#   "heart"          → Heart (1036)
#   "euler"          → Euler, uses roll/pitch/yaw
#   "speed_level"    → SpeedLevel, uses speed_level field

string command

# Boolean flag — used for toggle commands (hand_stand, free_bound, etc.)
bool flag

# Euler angle control (rad) — used when command == "euler"
float32 roll
float32 pitch
float32 yaw

# Speed level (0~2) — used when command == "speed_level"
int32 speed_level
