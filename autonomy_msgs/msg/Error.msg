# Unified error reporting for actions and services (nav2-style error_code + error_msg).

# --- System (9xxx) ---
uint16 NONE=0
uint16 UNKNOWN=9000
uint16 INVALID_REQUEST=9001      # Bad parameters or unknown task_id
uint16 TASK_CONFLICT=9002        # Preempted or concurrent task rejected
uint16 TIMEOUT=9003
uint16 TF_ERROR=9004
uint16 NOT_AVAILABLE=9005          # Busy, estop, or subsystem disabled

# --- Navigation (1xxx) ---
uint16 NAV_PLANNER_FAILED=1001
uint16 NAV_CONTROLLER_FAILED=1002
uint16 NAV_GOAL_INVALID=1003

# --- Follow (2xxx) ---
uint16 FOLLOW_TARGET_LOST=2001
uint16 FOLLOW_TARGET_INVALID=2002

# --- Waypoints (3xxx) ---
uint16 WP_NO_VALID_WAYPOINTS=3001
uint16 WP_MISSED_WAYPOINT=3002     # Timeout or could not reach pose

# --- Dock / recharge (4xxx) ---
uint16 DOCK_NOT_FOUND=4001
uint16 DOCK_NOT_VALID=4002
uint16 DOCK_STAGING_FAILED=4003
uint16 DOCK_ALIGN_FAILED=4004
uint16 DOCK_CHARGE_FAILED=4005

# --- Teleop (5xxx) ---
uint16 TELEOP_PREEMPTED=5001
uint16 TELEOP_TIMEOUT=5002

# --- Guided tour / exhibition (6xxx) ---
uint16 TOUR_INVALID=6001
uint16 TOUR_EXHIBIT_SKIPPED=6002
uint16 TOUR_WAIT_TIMEOUT=6003
uint16 TOUR_CROWD_BLOCKED=6004

# --- Safety (8xxx) ---
uint16 SAFETY_ESTOP=8001
uint16 SAFETY_OBSTACLE=8002

uint16 error_code                 # One of the constants above
string error_msg                  # Human-readable detail for UI/logs
