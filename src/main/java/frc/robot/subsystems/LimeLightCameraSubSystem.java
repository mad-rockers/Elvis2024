package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightCameraSubSystem extends SubsystemBase
{
    private final NetworkTable m_limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");

    //Angle info
    //tx -> angle left or right from the camera
    //ty -> angle up for down from the camera
    //ta -> area on screen the apriltag takes up
    //tv -> check whether target is valid
    private final NetworkTableEntry m_tx = m_limeLightTable.getEntry("tx");
    private final NetworkTableEntry m_ty = m_limeLightTable.getEntry("ty");
    private final NetworkTableEntry m_ta = m_limeLightTable.getEntry("ta");
    private final NetworkTableEntry m_tv = m_limeLightTable.getEntry("tv");

    //Target info
    private final NetworkTableEntry m_targetPoseRS = m_limeLightTable.getEntry("targetpose_robotspace");

    public double getXAngleOffset() {
    return m_tx.getDouble(0.0);
    }

    public double getYAngleOffset() {
        return m_ty.getDouble(0.0);
    }

    public double getXDistOffset() {
        return m_targetPoseRS.getDoubleArray(new double[6])[0];
    }

    public double getYDistOffset() {
        return m_targetPoseRS.getDoubleArray(new double[6])[1];
    }

    public double getZDistOffset() {
        return m_targetPoseRS.getDoubleArray(new double[6])[2];
    }

    public double getXRotation() {
        return m_targetPoseRS.getDoubleArray(new double[6])[3];
    }

    public double getYRotation() {
        return m_targetPoseRS.getDoubleArray(new double[6])[4];
    }

    public double getZRotation() {
        return m_targetPoseRS.getDoubleArray(new double[6])[5];
    }

    public double getVisualArea() {
        return m_ta.getDouble(0.0);
    }

    public boolean isTargetVisible() {
        return m_tv.getDouble(0) == 1;
    }






    public double getDistance2D() {
        double distance =
            Math.sqrt(getXDistOffset() * getXDistOffset() + getZDistOffset() * getZDistOffset());

        return distance;
    }
    public double getDistance3D() {
        double distance =
            Math.sqrt(
                getXDistOffset() * getXDistOffset()
                    + getYDistOffset() * getYDistOffset()
                    + getZDistOffset() * getZDistOffset());
        return distance;
    }



    @Override
    public void periodic() {
    SmartDashboard.putBoolean("Is Visible", isTargetVisible());
    SmartDashboard.putNumber("X Distance Offset: ", getXDistOffset());
    SmartDashboard.putNumber("Y Distance Offset: ", getYDistOffset());
    SmartDashboard.putNumber("Z Distance Offset: ", getZDistOffset());

    SmartDashboard.putNumber("X Angle Offset: ", getXRotation());
    SmartDashboard.putNumber("Y Angle Offset: ", getYRotation());
    SmartDashboard.putNumber("Z Angle Offset: ", getZRotation());

    SmartDashboard.putNumber("Distance 2D", getDistance2D());
    SmartDashboard.putNumber("Distance 3D", getDistance3D());
  }
}
