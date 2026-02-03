For Limelight3A:
- [Example code from the official site for Limelight3A](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorLimelight3A.java);
- [Limelight downloads (such as Limelight hardware manager, FTC field Map)](https://docs.limelightvision.io/docs/resources/downloads);
- [Limelight 3A Quick-Start](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a)
- [Esitmating Distance](https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance):
   - 1)using ty angle, limelight height and AprilTag height;
   - 2)Using the Apriltag area, [see example](https://www.youtube.com/watch?v=Ap1lBywv00M)
 
- [April Tag Guideline](https://ftc-docs-cdn.ftclive.org/booklets/en/april_tags.pdf?utm_source=chatgpt.com)
 
## Limelight (need to modify):
- [Limelight 3A Documentation](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a)
- [Limelight Coding-MegaTag2 with IMU](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
- [Distance Estimation and Angle Alignment](https://deep-blue-training.readthedocs.io/en/latest/section-7/limelight/)
- [Limelight AprilTag Aiming Demo](https://www.youtube.com/watch?v=-EfOzB_A00Q)
- [How to Measure Distance with April Tags (Limelight 3A)](https://www.youtube.com/watch?v=Ap1lBywv00M)


### To-do List (2/4/2026)
- Check Limelight setup (Limelight Hardware Manager and pipeline number)
- Double check the locations of the Blue and Red targets:
  - Tag 20 (Blue): x=-1.482, y=-1.413, z=0.749 (in meters) / x=-58.35, y=-55.63, z=29.49 (in inches)
  - Tag 24 (Red):  x=-1.482, y= 1.413, z=0.749 (in meters) /x=-58.35, y=55.63, z=29.49 (in inches);
  - ![FTC Decode Field Map](images/decode-field_CS.png)
- LL_LENS_HEIGHT_IN = 10.5;     // camera lens center above tile (need to measure)
- In "applyFlywheelControl()", should we move "double currentRPM = mFW.getVelocity() * 60.0 / TICKS_PER_REV;" after "mFW.setVelocity(targetTicksPerSec)"? 
