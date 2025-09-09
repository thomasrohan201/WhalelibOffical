A fork of Lemlib with the following features [you do need Lemlib and Pros backend files for this to work]:
1. Relative motion
ex.     whale.moveBackward(float inches, int timeout);
        whale.moveForward(float inches, int timeout);

   
3. Gain scheduler with interpolation
(seen in initialize)
ex.     whaleFastScheduler.moveToPointWithScheduler(float x, float y, int timeout);


5. Dynamic PID use
ex.     whalefast.moveToPoint(float x, float y, int timeout);
        whaleAccurate.moveToPoint(float x, float y, int timeout);

   
8. Reset Pose
ex.     whale.resetPose(lemlib::Pose alignmentPoint, float alignmentDistance, int imu_port)


10. Reset Position
ex.     whale.resetPositionOnly(lemlib::Pose alignmentPoint, float alignmentDistance)


Real Documentation coming soon; however, to get started, the Lemlib documentation should take you far enough:
https://lemlib.readthedocs.io/en/stable/
