package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoMotorDriveSystem extends SubsystemBase 
{

  ///CONSTANTS///
  private static boolean debugMode = true;

  // IDs
  private static final int frontLeft_ID  = 1;
  private static final int frontRight_ID = 3;
  private static final int backLeft_ID   = 2;
  private static final int backRight_ID  = 4;

  // Invert
  private static final boolean invertLeft  = true;
  private static final boolean invertRight = false;

  // Motors
  private CANSparkMax m_frontLeftMotor;
  private CANSparkMax m_frontRightMotor;
  private CANSparkMax m_backLeftMotor;
  private CANSparkMax m_backRightMotor;  

  private final RelativeEncoder m_frontLeftEncoder;
  private final RelativeEncoder m_frontRightEncoder;
  private final RelativeEncoder m_backLeftEncoder;
  private final RelativeEncoder m_backRightEncoder;


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




    ///DIFFERENTIAL DRIVE///
    m_drive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);
    
  }


  ///METHODS///
  //TODO: Inputs Curves, Deadzones
  public void driveArcade(double speed, double rotation)
  {
    m_drive.arcadeDrive(speed, rotation);
  }


  public void driveTank(double speedLeft, double speedRight)
  {
    m_drive.tankDrive(speedLeft, speedRight);
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
