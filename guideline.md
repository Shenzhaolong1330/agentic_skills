Use the skills in this directory to do this task:

Pick tubes from the table and insert them into black rack on the table

Guidelines:
1. Follow the procedure in /home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/run_single_pick_tube_insert_rack.sh First locate and grasp tail to head 1/5 point. Then finish transition. Next locate the rack and go to observation pose before locate the empty hole on rack. Finally move to the empty hole and insert the tube.
2. Check head realsense camera's rgb images each step to make sure the process is running normally.
3. After a complete insertion, use `/home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_go_home.sh --mode live --hardware-allowed --execute` to go home and prepare for the next tube insertion.
4. If the robot encounters an obstacle or error, do not auto-clear E-stop or an unsafe state. After manual safety confirmation, use `/home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_reset.sh --mode live --hardware-allowed --execute` only when a physical reset is appropriate.
5. Don't stop until you finish the task.
6. You can change object-locator's config to suit your task.
