package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Units;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TankDrive extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  public final PIDController pid = new PIDController(0.00000003397,0, 0);//0.0003397
  public final PIDController limelightpid = new PIDController(0.05,0,0);//0.0003397
  public final PIDController drivingpid = new PIDController(0.2,0,0);
  public PowerDistributionPanel pdp = new PowerDistributionPanel();
  //public static final AHRS ahrs = new AHRS();

  

  //Odometry class for tracking robot pose
  //private final DifferentialDriveOdometry m_odometry;

  //Diameter of tankDriveWheels in meters
  private final double wheelDiameter = Units.inchesToMeters(6);
  
  public TankDrive() {
    setDefaultCommand(new DriveCommand(this));
    pid.setTolerance(128); //Error is within 1/8 of a revolution
    

    //resetEncoders();
    //m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public double fps(double input) {

    return ((input*10 / 1024 ) / (6*Math.PI))/12;
  }

  public void driveWithJoystick() {
    //ONE JOYSTICK
    
    //double joysticky = Math.signum(RobotContainer.stick.getY())* Math.pow(RobotContainer.stick.getY(), 2);
    //double forward = drivingpid.calculate(fps(RobotContainer.middleLeft.getSelectedSensorVelocity()), joysticky*30);

    //double forward = .8*MathUtil.clamp(1 * Math.pow(RobotContainer.stick.getY(), 3),-1,1);
    //double forward = Math.signum(RobotContainer.stick.getY())* (Math.pow(RobotContainer.stick.getY(), 2));
    //double forward = Math.signum(RobotContainer.stick.getY())* Math.abs(Math.log10(RobotContainer.stick.getY()));
    double forward = .85*RobotContainer.stick.getY();

    //double forward = RobotContainer.stick.getY()]

    SmartDashboard.putNumber("Forward", forward);
    //double turn = .7*MathUtil.clamp(-1 * Math.pow(RobotContainer.stick.getZ(), 3),-1,1);
    //double turn = Math.pow(RobotContainer.stick.getZ(), 2);
    //double turn =  Math.signum(RobotContainer.stick.getZ())* (Math.pow(RobotContainer.stick.getZ(), 2));
    double turn = .7*RobotContainer.stick.getZ();

    //double joystickz = Math.signum(RobotContainer.stick.getY())* Math.pow(RobotContainer.stick.getZ(), 2);
    //double turn = drivingpid.calculate(fps(RobotContainer.middleLeft.getSelectedSensorVelocity()), joystickz*30);

    

    SmartDashboard.putNumber("Turn", turn);

    /*deadband*/
    /*
    if (Math.abs(forward) < 0.05)
    {
      forward = 0;
    } 
    
    if (Math.abs(turn) < 0.05) {
      turn = 0;
    }
    */
    forward = MathUtil.clamp(forward, -.85,.85)*-RobotContainer.stick.getThrottle();
    turn = MathUtil.clamp(turn, -.7,.7)*Math.abs(RobotContainer.stick.getThrottle());

    RobotContainer.difDrive.arcadeDrive(forward, -turn);
    SmartDashboard.putNumber("motor Voltage", RobotContainer.middleLeft.get()); 
  }
  
  

  public void stop() {
    RobotContainer.difDrive.arcadeDrive(0, 0);
  }
  
  public void turn(boolean isLeft){
    if(isLeft){
      RobotContainer.difDrive.tankDrive(-.05, .05);
    }else{
      RobotContainer.difDrive.tankDrive(.05, -.05);
    }
  }

  public void move(double forward, double turn) {
    RobotContainer.difDrive.arcadeDrive(forward, turn);
  }

  /*
  public void limelightAlign() {
    double turnBy = MathUtil.clamp(limelightpid.calculate(RobotContainer.limelight.getXOffset(),0),-.4,.4);
    double forward = MathUtil.clamp(limelightpid.calculate(RobotContainer.limelight.getYOffset(),0),-.4,.4);
    if(Math.abs(RobotContainer.limelight.getXOffset()) < 10){
      turnBy = -Math.signum(RobotContainer.limelight.getXOffset())*.28;
    }
    if(Math.abs(RobotContainer.limelight.getXOffset()) < 1){
      turnBy = 0;
    }
    if(Math.abs(RobotContainer.limelight.getYOffset()) < 10){
      forward =  Math.signum(RobotContainer.limelight.getYOffset())*.28;
    }
    if(Math.abs(RobotContainer.limelight.getYOffset()) < 1){
      turnBy = 0;
    }
    SmartDashboard.putNumber("turnBy", turnBy);
    RobotContainer.difDrive.tankDrive(turnBy , -turnBy);
  }
  */



  /*
  public void driveDistance(double distance) {
    double count = -distance * (2048 / (6 * Math.PI));
    RobotContainer.middleLeft.setSelectedSensorPosition(0);
    pid.setSetpoint(count);
    
    
  }

  
  

  public void updateDrive() {
    double output = pid.calculate(RobotContainer.middleLeft.getSelectedSensorPosition(), pid.getSetpoint());
    SmartDashboard.putNumber("count", pid.getSetpoint());
    SmartDashboard.putNumber("Pid Output", output);
    SmartDashboard.putNumber("current count", RobotContainer.middleLeft.getSelectedSensorPosition());
    /*
    if(output > 0.4) {
      output = 0.4;
    }

    if(output < -0.4){
      output = -0.4;
    }
    *
    RobotContainer.difDrive.arcadeDrive(output, 0);
    SmartDashboard.putNumber("Total auto current", pdp.getTotalCurrent());
  }*/
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //m_odometry.update(Rotation2d.fromDegrees(getHeading()), 
    //getActualDistance(RobotContainer.middleLeft); 
    //getActualDistance(RobotContainer.middleRight);
  }

  
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  /*
   public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }*/

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   *
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getActualRate(RobotContainer.middleLeft), getActualRate(RobotContainer.middleRight));
  }*/

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  /*
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

   /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   *
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    RobotContainer.leftSide.setVoltage(leftVolts);
    RobotContainer.rightSide.setVoltage(rightVolts);
    RobotContainer.difDrive.feed();
  }*/

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  /*
  public void resetEncoders() {
    RobotContainer.middleLeft.setSelectedSensorPosition(0);
    RobotContainer.middleRight.setSelectedSensorPosition(0);
  }

   /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   
  public double getAverageEncoderDistance() {
    return (getActualDistance(RobotContainer.middleLeft) + getActualDistance(RobotContainer.middleRight)) / 2.0;
  }*/

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    RobotContainer.difDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   
  public void zeroHeading() {
    ahrs.reset();
  }*/

  /** 
   * Gets the actual distance in meters traveled by the robot based on encoder values
   * 
   * @param motor The TalonSRX motor with an encoder
   * 
   * @return The distance traveled by the robot in meters 
  */
  public double getActualDistance(WPI_TalonFX motor) {
    double count = motor.getSelectedSensorPosition();
    return ((wheelDiameter * Math.PI)/1024) * count;
  }

  /**
   * Gets the actual rate in meters per second traveled by the robot
   * 
   * @param motor The TalonSRX motor with an encoder
   * 
   * @return The rate at which the robot travels in meters per second
   */
  public double getActualRate(WPI_TalonFX motor) {
    double encoderRate = motor.getSelectedSensorVelocity(); //Returns count per 100ms
    return (10 * encoderRate * wheelDiameter * Math.PI) / 1024.0;
  }

   /*
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   
   public double getHeading() {
     return ahrs.getYaw();
   }*/
   
   /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   
  public double getTurnRate() {
    return ahrs.getRate();
  }
  */

}