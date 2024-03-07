package frc.robot.subsystems;

import java.lang.Math;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.RobotConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoMotorDriveSystem extends SubsystemBase 
{

  ///CONSTANTS///
  private static boolean debugMode = true;

  // IDs
  private static final int frontLeft_ID  = 2;
  private static final int frontRight_ID = 4;
  private static final int backLeft_ID   = 3;
  private static final int backRight_ID  = 5;

  // Invert
  private static final boolean invertLeft  = true;
  private static final boolean invertRight = false;

  // Motors
  private CANSparkMax m_frontLeftMotor;
  private CANSparkMax m_frontRightMotor;
  private CANSparkMax m_backLeftMotor;
  private CANSparkMax m_backRightMotor;  

  // Encoders
  private final RelativeEncoder m_frontLeftEncoder;
  private final RelativeEncoder m_frontRightEncoder;
  private final RelativeEncoder m_backLeftEncoder;
  private final RelativeEncoder m_backRightEncoder;


  // PID Controllers
  private final SparkPIDController m_frontLeftPID;
  private final SparkPIDController m_frontRightPID;

  // Previous Wheel Rotations
  private double prevLeftRotations = 0;
  private double prevRightRotations = 0;
    

  // Differential Drive
  private DifferentialDrive m_drive;

  public NeoMotorDriveSystem()
  {
    ///CREATE MOTORS///
    m_frontLeftMotor = new CANSparkMax(frontLeft_ID,  MotorType.kBrushless);
    m_frontRightMotor = new CANSparkMax(frontRight_ID, MotorType.kBrushless);
    m_backLeftMotor = new CANSparkMax(backLeft_ID,   MotorType.kBrushless);
    m_backRightMotor = new CANSparkMax(backRight_ID,  MotorType.kBrushless);

    

    ///SET-UP MOTORS///

    //Default PIDs
    m_frontLeftMotor.restoreFactoryDefaults();
    m_frontRightMotor.restoreFactoryDefaults();
    m_backLeftMotor.restoreFactoryDefaults();
    m_backRightMotor.restoreFactoryDefaults();

    //Followers
    m_backLeftMotor.follow(m_frontLeftMotor);
    m_backRightMotor.follow(m_frontRightMotor);

    //Inverts
    m_frontLeftMotor.setInverted(invertLeft);
    m_frontRightMotor.setInverted(invertRight);


    ///ENCODERS///
    m_frontLeftEncoder  = m_frontLeftMotor.getEncoder();
    m_frontRightEncoder = m_frontRightMotor.getEncoder();
    m_backLeftEncoder   = m_backLeftMotor.getEncoder();
    m_backRightEncoder  = m_backRightMotor.getEncoder();

      //PID Controllers
      m_frontLeftPID  = m_frontLeftMotor.getPIDController();
      m_frontRightPID = m_frontRightMotor.getPIDController();



    ///DIFFERENTIAL DRIVE///
    m_drive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);

    //Initiates SmartDashboard Drive Type
    SmartDashboard.putString("Drive Type", "ArcadeDrive");
    
  }
  ///INTERNAL METHODS///
  private double applyDeadBand(double inp)
  {
    if(Math.abs(inp)>(0.1)){ // Control deadband here
      return inp;
    }
    return 0.0;
  }

  private double dampenSpeed(double inp){
    return(Math.abs(inp)*inp*0.5); // Control speeddampener here
  }

  ///EXTERNAL METHODS///
  public void driveArcade(double speed, double rotation)
  {
    m_drive.arcadeDrive(dampenSpeed(applyDeadBand(speed)), dampenSpeed(applyDeadBand(rotation)));
  }


  public void driveTank(double speedLeft, double speedRight)
  {
    m_drive.tankDrive(dampenSpeed(applyDeadBand(speedLeft)), dampenSpeed(applyDeadBand(speedRight)));
  }

  public void setMotorRPM(double rpm)
  {
    double setPoint = rpm;
    m_frontLeftPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    m_frontRightPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }


  public void stopMotors() 
  {
    m_frontLeftMotor.set(0);
    m_frontRightMotor.set(0);
  }

  public void resetDistanceTraveled()
  {
    prevLeftRotations = m_frontLeftEncoder.getPosition();
    prevRightRotations = m_frontRightEncoder.getPosition();
  }

  public double getDistanceTraveled()
  {
    //Get number of rotations
    double currentLeftRotations = m_frontLeftEncoder.getPosition();
    double currentRightRotations = m_frontRightEncoder.getPosition();

    //Distances in inches
    double leftDistance = (currentLeftRotations-prevLeftRotations) * 2 * Math.PI * RobotConstants.WHEEL_RADIUS_IN;
    double rightDistance = (currentRightRotations-prevRightRotations) * 2 * Math.PI * RobotConstants.WHEEL_RADIUS_IN;
    
    //Reset Counts
    prevLeftRotations = currentLeftRotations;
    prevRightRotations = currentRightRotations;

    double averageDistance = (leftDistance+rightDistance)/2;

    return averageDistance;
  }

  ///DEBUG INFO///
  @Override
  public void periodic() 
  {
    if (debugMode)
    {
      SmartDashboard.putNumber("FL Motor Velocity", m_frontLeftEncoder.getVelocity());
      SmartDashboard.putNumber("FR Motor Velocity", m_frontRightEncoder.getVelocity());
      SmartDashboard.putNumber("BL Motor Velocity", m_backLeftEncoder.getVelocity());
      SmartDashboard.putNumber("BR Motor Velocity", m_backRightEncoder.getVelocity());
    }
  }
}
