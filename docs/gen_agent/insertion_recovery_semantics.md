# Insertion recovery semantics

The migrated insertion workflow treats insertion as a verified physical goal,
not as successful completion of a motion command. Its final predicate
requires fresh evidence that the tube is inside the rack, supported by the
rack, no longer held by the gripper, and that the robot is healthy/ready.

Insertion failure follows this bounded order where policy permits:

1. invalidate stale pose, occupancy, holding, and release facts;
2. safe retreat or re-observe the scene;
3. select an alternate hole/arm/candidate when deterministic evidence allows;
4. recompile only the remaining graph under the original envelope; and
5. request a human or abort when evidence is ambiguous, the object may be
   dropped, an emergency state is present, or the budget is exhausted.

The workflow never retries insertion by lifting the robot and repeating the
same insertion parameters. A release command is not release evidence, and a
successful reset is not proof that the object is still held or correctly
placed. The offline dispatcher returns plans and fixture observations only;
`physical_goal_verified` and `physical_execution_performed` remain false.
