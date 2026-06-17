# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");

# test_planner / test_controller require optional autonomy modules
# (mppi_controller, navigator action_type) not shipped in this tree.
message(STATUS "Skipping autonomy_ros integration tests (optional headers unavailable)")
