# ID for obstacle association
int32 id

# Current pose of the obstacle
geometry_msgs/Pose pose

# List of Gaussians and their probabilities
pedsim_msgs/Gaussian[] gaussians
float64[] probabilities
