package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
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

  // PID Controllers
  private final SparkPIDController m_frontLeftPID;
  private final SparkPIDController m_frontRightPID;

  private final RelativeEncoder m_frontLeftEncoder;
  private final RelativeEncoder m_frontRightEncoder;
  private final RelativeEncoder m_backLeftEncoder;
  private final RelativeEncoder m_backRightEncoder;


  // Differential Drive
  private DifferentialDrive m_drive;



  //Double check access modifiers
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setPoint;

  // kP = 0;
  // kI = 0;
  // kD = 0;
  // kIz = 0;
  // kFF = 0.000215;
  // kMaxOutput = 1;
  // kMinOutput = -1;
  // maxRPM = 150;

  private void setPIDCoeffients(SparkPIDController m_pidController)
  {
    // Set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  private void DefaultPIDCoeffients()
  {
    // PID coefficients
    kP = 0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000215;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 150;
  }

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


    ///PID CONTROLLERS///
    m_frontLeftPID = m_frontLeftMotor.getPIDController();
    m_frontRightPID = m_frontLeftMotor.getPIDController();

    DefaultPIDCoeffients();

    setPIDCoeffients(m_frontLeftPID);
    setPIDCoeffients(m_frontRightPID);

    ///DIFFERENTIAL DRIVE///
    m_drive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);
    
  }


  ///METHODS///
  // Method to set motor speed
  public void setMotorSpeed(double speed) 
  {
    m_frontLeftMotor.set(speed);
    m_frontRightMotor.set(speed);
  }

  // Method to set motors to a specific RPM
  public void setMotorRPM(double rpm) {
    setPoint = rpm;
    m_frontLeftPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    m_frontRightPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  // Example usage to set motors to 300 RPM
  public void setMotorTo150RPM() {
    setMotorRPM(150);
  }

  public void manualMotorControl(double stickInput) {
    // 5% range for dead stick
    double percentageOfFullRPM = stickInput;
    if (stickInput >= -0.05 && stickInput <= 0.05) {
      percentageOfFullRPM = 0;
    }
    setMotorRPM(percentageOfFullRPM * maxRPM);
  }

    // Method to stop all motors
  public void stopMotors() 
  {
    m_frontLeftMotor.set(0);
    m_frontRightMotor.set(0);
  }

  //TODO: Inputs Curves, Deadzones
  public void driveArcade(double joyXAxis, double joyYAxis)
  {
    m_drive.arcadeDrive(joyXAxis, joyYAxis);
  }


  public void driveTank(double joyXAxis, double joyYAxis)
  {
    m_drive.tankDrive(joyXAxis, joyYAxis);
  }



  ///DEBUG INFO///
  @Override
  public void periodic() 
  {
    if (debugMode)
    {
      // Read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);

      // If PID coefficients on SmartDashboard have changed, write new values to controller
      if ((p != kP)) {
        m_frontLeftPID.setP(p);
        m_frontRightPID.setP(p);

        kP = p;
      }
      if ((i != kI)) {
        m_frontLeftPID.setI(i);
        m_frontRightPID.setI(i);

        kI = i;
      }
      if ((d != kD)) {
        m_frontLeftPID.setD(d);
        m_frontRightPID.setD(d);

        kD = d;
      }
      if ((iz != kIz)) {
        m_frontLeftPID.setIZone(iz);
        m_frontRightPID.setIZone(iz);

        kIz = iz;
      }
      if ((ff != kFF)) {
        m_frontLeftPID.setFF(ff);
        m_frontRightPID.setFF(ff);

        kFF = ff;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
        m_frontLeftPID.setOutputRange(min, max);
        m_frontRightPID.setOutputRange(min, max);

        kMinOutput = min;
        kMaxOutput = max;
      }

      SmartDashboard.putNumber("SetPoint", setPoint);
      SmartDashboard.putNumber("FL Motor Velocity", m_frontLeftEncoder.getVelocity());
      SmartDashboard.putNumber("FR Motor Velocity", m_frontRightEncoder.getVelocity());
      SmartDashboard.putNumber("BL Motor Velocity", m_backLeftEncoder.getVelocity());
      SmartDashboard.putNumber("BR Motor Velocity", m_backRightEncoder.getVelocity());

    }
  }
}
