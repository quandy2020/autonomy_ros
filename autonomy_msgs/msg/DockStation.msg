# Charging dock definition (ListDocks / Dock action).

string dock_id                 # Unique charger id
string dock_type               # Vendor/model tag, free-form
geometry_msgs/PoseStamped dock_pose      # Final dock / charge pose
geometry_msgs/PoseStamped staging_pose   # Optional approach pose before align
