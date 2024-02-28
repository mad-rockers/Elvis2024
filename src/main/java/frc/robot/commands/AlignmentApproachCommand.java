package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightCameraSubSystem;
import frc.robot.subsystems.NeoMotorDriveSystem;

public class AlignmentApproachCommand extends Command {

  private final NeoMotorDriveSystem m_driveSubsystem;
  private final LimeLightCameraSubSystem m_cameraSubsystem;

  private final Timer m_timer = new Timer();
  private static final double MAX_RUN_TIME_SECONDS = 5;

  // TODO: Refactor the way the step size is created, otherwise you end up with Zeno's Paradox
  private static final double STEP_SIZE = 0.2;
  private static final double STEP_SPEED_S = 1;
  private static final double FINAL_TARGET_DISTANCE_M = 0.3048;//m
  private static final double TURN_RADIUS_CM = 27.305;//cm
  private static final double WHEEL_RADIUS_CM = 7.62;//cm

  // private static final double APPROACH_SPEED = 0.5;

  public AlignmentApproachCommand(
      NeoMotorDriveSystem driveSubsystem, LimeLightCameraSubSystem cameraSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_cameraSubsystem = cameraSubsystem;

    addRequirements(m_driveSubsystem, m_cameraSubsystem);
  }


  public double GetAngleOffset(double tagXPosition, double tagYPosition, double tagZRotation)
  {
      double interceptDistance = 0;

      // If the tag's rotation ends up being 0 then we'll get a div 0 error.
      // Luckily, we should just be able to set the intercept distance to the xPosition of the
      // aprilTag in this case
      try {
        interceptDistance =
            (tagXPosition * Math.tan(Math.toRadians(tagZRotation)) - tagYPosition)
                / (-1 * Math.tan(Math.toRadians(tagZRotation)));
      } catch (Exception e) {
        interceptDistance = tagXPosition;
      }

      double trajectoryPointX =
          ((-2 * STEP_SIZE * interceptDistance)
                  + (2 * interceptDistance)
                  + (STEP_SIZE * tagXPosition))
              * STEP_SIZE;
      double trajectoryPointY = (STEP_SIZE * STEP_SIZE * tagYPosition);

      double angleOfCorrection = Math.toDegrees(Math.atan(trajectoryPointY / trajectoryPointX));

      return angleOfCorrection;
  }

  public double GetTrajectoryPointDistance(double tagXPosition, double tagYPosition, double tagZRotation)
  {
      double interceptDistance = 0;
      
      // If the tag's rotation ends up being 0 then we'll get a div 0 error.
      // Luckily, we should just be able to set the intercept distance to the xPosition of the
      // aprilTag in this case
      try {
        interceptDistance =
            (tagXPosition * Math.tan(Math.toRadians(tagZRotation)) - tagYPosition)
                / (-1 * Math.tan(Math.toRadians(tagZRotation)));
      } catch (Exception e) {
        interceptDistance = tagXPosition;
      }

      double trajectoryPointX =
          ((-2 * STEP_SIZE * interceptDistance)
                  + (2 * interceptDistance)
                  + (STEP_SIZE * tagXPosition))
              * STEP_SIZE;
      double trajectoryPointY = (STEP_SIZE * STEP_SIZE * tagYPosition);

      double trajectoryPointDistance = Math.sqrt(trajectoryPointX*trajectoryPointX+trajectoryPointY*trajectoryPointY);
      return trajectoryPointDistance;
  }

  public double getTurnRate(double angleOfCorrection)
  {
    double wheelRPM = 0;


    double wheelCircumference = 2*Math.PI*WHEEL_RADIUS_CM; //cm

    double arcLength = (angleOfCorrection/360)*2*Math.PI*TURN_RADIUS_CM; //cm
    SmartDashboard.putNumber("Arc Length", arcLength);
    wheelRPM = 60 * arcLength / wheelCircumference; //rpm
    return wheelRPM;
  }
  public double getDistanceRate(double distance)
  {
    double wheelRPM = 0;

    double wheelCircumference = 2*Math.PI*(WHEEL_RADIUS_CM*100); //cm
    
    wheelRPM = (60*distance)/(STEP_SPEED_S * wheelCircumference);
    return wheelRPM;
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    if (m_cameraSubsystem.isTargetVisible()) {
      double tagXPosition = m_cameraSubsystem.getXDistOffset();
      double tagYPosition = m_cameraSubsystem.getYDistOffset();
      double tagZRotation = m_cameraSubsystem.getZRotation();
      double pointDistance = GetTrajectoryPointDistance(tagXPosition, tagYPosition, tagZRotation);

      double angleOfCorrection = GetAngleOffset(tagXPosition, tagYPosition, tagZRotation);
      double turnRate = getTurnRate(angleOfCorrection);
      double moveRate = getDistanceRate(pointDistance);

      SmartDashboard.putNumber("Correction Angle:", angleOfCorrection);
      SmartDashboard.putNumber("Turn Rate:", turnRate);
      SmartDashboard.putNumber("Move Rate:", moveRate);
      // double activeTimer = m_timer.get();
      // while (activeTimer < m_timer.get()+1)
      // {
      //   m_driveSubsystem.setRotationRPM(turnRate);
      // }
      // m_driveSubsystem.stopMotors();
      // activeTimer = m_timer.get();
      // while (activeTimer < m_timer.get()+STEP_SPEED_S)
      // {
      //   m_driveSubsystem.setMotorRPM(moveRate);
      // }

      // TODO: Some sort of method to convert the angle of correction and speed to useable motor
      // inputs

      // m_driveSubsystem.driveArcade(alignmentSpeed, alignmentAdjustment);

    }
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(MAX_RUN_TIME_SECONDS)
        || (m_cameraSubsystem.getDistance2D() <= (FINAL_TARGET_DISTANCE_M));
  }

  @Override
  public void end(boolean interruped) {
    m_driveSubsystem.driveArcade(0.0, 0.0);
    m_timer.stop();
  }
}
