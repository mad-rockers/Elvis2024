package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.Math;

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

  private final RelativeEncoder m_frontLeftEncoder;
  private final RelativeEncoder m_frontRightEncoder;
  private final RelativeEncoder m_backLeftEncoder;
  private final RelativeEncoder m_backRightEncoder;


  // Differential Drive
  private DifferentialDrive m_drive;

  // Drive Type
  private int driveType = 0;  

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

  public void changeDrive(){
    if(driveType == 0){
      driveType = 1;
      return;
    }
    if(driveType == 1){
      driveType = 0;
      return;
    }else{
      driveType = 0;
      return;
    }
  }

  public double[] deadBand(double[] bits, double band){
    double[] viable = new double[bits.length];
    int ind = 0;
    for(double num:bits){
      if(Math.abs(num)-band > 0){
        viable[ind] = num;
      }
    }
    return viable;
  }

  public void driveAll(double speedNspeedLeft, double rotation, double speedRight){
    double[] vals = {speedNspeedLeft,rotation,speedRight};
    vals = deadBand(vals,0.1);
    if(driveType == 0){
      m_drive.arcadeDrive(vals[0]*Math.abs(vals[0])*0.5, vals[1]*Math.abs(vals[1])*0.5); //speedNspeedLeft, rotation
    }
    if(driveType == 1){
      m_drive.tankDrive(vals[0]*Math.abs(vals[0])*0.5, vals[2]*Math.abs(vals[2])*0.5); //speedNspeedLeft, speedRight
    }
  }

/*
  public void driveArcade(double speed, double rotation)
  {
    m_drive.arcadeDrive(speed*Math.abs(speed)*0.5, rotation*Math.abs(rotation)*0.5);
  }


  public void driveTank(double speedLeft, double speedRight)
  {
    m_drive.tankDrive(speedLeft*Math.abs(speedLeft)*0.5, speedRight*Math.abs(speedRight)*0.5);
  }
*/

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
