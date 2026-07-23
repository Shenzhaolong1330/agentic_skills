# Transform pose

`geometry.transform_pose` is pure computation. It accepts an explicit source
frame, target frame, fixed transform chain, and expected calibration hash. It
rejects missing/ambiguous frames, non-finite values, and calibration mismatch;
it never guesses that a camera frame is a robot base frame or reads a file path
provided by a caller.
