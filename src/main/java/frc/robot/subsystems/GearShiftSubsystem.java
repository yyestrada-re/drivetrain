package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class GearShiftSubsystem extends SubsystemBase {
    //Creates solenoid object
    //hi its me, natarichard :) ٩(♡ε♡ )۶ 

    public GearShiftSubsystem() {  
      //in();
        //setDefaultCommand(new GetPressure(this));
    }

    public void out() {  
        RobotContainer.gearshift1.set(true);
        RobotContainer.gearshift2.set(true);
        SmartDashboard.putString("GearShift","Out");
      }
      
      public void in() {
        RobotContainer.gearshift1.set(false);
        RobotContainer.gearshift2.set(false);
        SmartDashboard.putString("GearShift","In");
      }
}
