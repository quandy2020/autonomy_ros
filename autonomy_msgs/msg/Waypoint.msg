# One stop in NavigateThrough (or reusable in custom tour definitions).

string id                      # Client-defined waypoint id
string label                   # Display name
geometry_msgs/PoseStamped pose
float32 wait_duration 0.0      # Seconds to wait after arrival before next wp
string narration_id            # Optional TTS/AV content id at this stop
bool wait_for_continue false   # If true, external ContinueTour-like signal needed
