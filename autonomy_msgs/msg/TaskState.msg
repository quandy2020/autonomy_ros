# Lifecycle state of a long-running autonomy task.

uint8 UNKNOWN=0
uint8 ACCEPTED=1          # Goal accepted, not yet executing
uint8 RUNNING=2
uint8 SUCCEEDED=3
uint8 FAILED=4
uint8 CANCELED=5          # User cancel or preempted (see TASK_CONFLICT)
uint8 PAUSED=6            # PauseTask engaged

uint8 value
