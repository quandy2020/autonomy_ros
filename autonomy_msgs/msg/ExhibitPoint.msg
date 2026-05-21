# Exhibition stop for GuidedTour: navigation target + narration trigger.

string exhibit_id              # Unique id within tour
string exhibit_name            # Display / log name
string narration_id            # Content id for AV or cloud playback on arrival
geometry_msgs/PoseStamped pose
float32 dwell_duration 30.0    # Auto-leave after seconds; 0 = no auto timer
bool wait_for_continue false   # If true, call ContinueTour srv before leaving
float32 xy_tolerance 0.25      # Arrival radius override (meters)
float32 yaw_tolerance 0.35     # Reserved for future heading check (radians)
