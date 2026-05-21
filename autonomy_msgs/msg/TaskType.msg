# Category of the active external command (mirrors which action is running).

uint8 IDLE=0
uint8 NAVIGATION=1        # NavigatePose
uint8 FOLLOW=2
uint8 WAYPOINTS=3         # NavigateThrough
uint8 DOCK=4
uint8 TELEOP=5
uint8 GUIDED_TOUR=6       # Exhibition hall guided tour (展厅导览)

uint8 value
