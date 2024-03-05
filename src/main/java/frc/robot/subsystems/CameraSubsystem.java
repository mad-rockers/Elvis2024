package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraSubsystem extends SubsystemBase {

    /*
     * Constant heights for various parts of the distance equation. 
     */
    private static final double CAMERA_HEIGHT = 3.0; // Height of the camera from the ground in inches
    private static final double TARGET_HEIGHT = 52.0; // Height of the target from the ground in inches
    private static final double CAMERA_ANGLE = 10.0; // Angle of the camera in degrees

    // NetworkTable entries for Limelight
    private final NetworkTable limelightTable;
    /*
     * tv = whether or not the camera sees a valid target.
     */
    private NetworkTableEntry tv;
    /*
     * ty = the vertical offset, in degrees. 
     */
    private NetworkTableEntry ty;

    public CameraSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelightTable.getEntry("tv");
        ty = limelightTable.getEntry("ty");
    }

    public boolean isValidTargetSeen() {
        return tv.getDouble(0.0) == 1.0;
    }

    public double calculateDistanceToTarget() {
        if (!isValidTargetSeen()) {
            /*
             * Return a negative number as another indicator that the target is not visible. 
             */
            return -1; 
        }

        double targetOffsetAngleVertical = ty.getDouble(0.0);
        /*
         * Adjust calculation to account for the vertical angle of the camera. 
         */
        double angleToTargetRad = Math.toRadians(targetOffsetAngleVertical + CAMERA_ANGLE);

        /*
         * Account for the height difference between the camera and the target, not just the target's height from the ground. 
         */
        double heightDifference = TARGET_HEIGHT - CAMERA_HEIGHT;
        
        /*
         * distance = height / tan(angle)
         */
        return heightDifference / Math.tan(angleToTargetRad);
    }

    @Override
    public void periodic() {
        /*
         * Can the target be seen?
         */
        SmartDashboard.putBoolean("Can See Target?", isValidTargetSeen());
        /*
         * Display the distance (in inches) to the target. If there is no target visible, then this should display a "-1".
         */
        SmartDashboard.putNumber("Distance to Target:", calculateDistanceToTarget());
    }
}
