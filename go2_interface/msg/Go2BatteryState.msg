# Go2BatteryState.msg
# Battery state from Unitree GO2 BmsState

float32 voltage      # Battery voltage (V)
float32 current      # Battery current (A), positive = charging
uint8   soc          # State of charge (0-100 %)
bool    is_charging  # True if charging
