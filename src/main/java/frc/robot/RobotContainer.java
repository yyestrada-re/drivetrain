// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.gearshift.GearShiftCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GearShiftSubsystem;
import frc.robot.subsystems.TankDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static final TankDrive tankDriveSubsystem = new TankDrive(); 

  public static final GearShiftSubsystem m_gearshiftsubsystem = new GearShiftSubsystem();

  

   //TANK DRIVE MOTORS
  public static final WPI_TalonFX frontLeft = new WPI_TalonFX(15); //0
  public static final WPI_TalonFX middleLeft = new WPI_TalonFX(13); //13
  public static final WPI_TalonFX rearLeft = new WPI_TalonFX(11); //11

  public static final WPI_TalonFX frontRight = new WPI_TalonFX(12); //12
  public static final WPI_TalonFX middleRight = new WPI_TalonFX(14); //1
  public static final WPI_TalonFX rearRight = new WPI_TalonFX(10);

  public static final SpeedControllerGroup leftSide = new SpeedControllerGroup(frontLeft, middleLeft, rearLeft);
  public static final SpeedControllerGroup rightSide = new SpeedControllerGroup(frontRight, middleRight, rearRight);

  //public static final SpeedControllerGroup leftSide = new SpeedControllerGroup(frontLeft, middleLeft);
  //public static final SpeedControllerGroup rightSide = new SpeedControllerGroup(frontRight, middleRight);

  public static final DifferentialDrive difDrive = new DifferentialDrive(leftSide, rightSide);

   //GEARSHIFT SOLENOIDS
  public static final Solenoid gearshift1 = new Solenoid(4);
  public static final Solenoid gearshift2 = new Solenoid(5);
  
  public static final Joystick stick = new Joystick(0);

  public static JoystickButton GearShiftButton = new JoystickButton(stick,1);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    GearShiftButton.whileHeld(new GearShiftCommand(m_gearshiftsubsystem), true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
