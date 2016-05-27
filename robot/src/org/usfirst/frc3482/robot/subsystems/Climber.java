package org.usfirst.frc3482.robot.subsystems;

import org.usfirst.frc3482.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final Talon climber = RobotMap.climber;
    private final Talon climberExtender = RobotMap.climberExtender;
    private final Talon climberRotator = RobotMap.climberRotator;
    
    public Climber() {
    	
    }
    
    public void startClimb() {
    	climber.set(1);
    }

    public void stopClimb() {
    	climber.set(0);
    }
    
    public void startExtendClimber() {
    	climberExtender.set(.75);
    }
    
    public void startRetractClimber() {
    	climberExtender.set(-0.75);
    }
    
    public void stopExtendClimber() {
    	climberExtender.set(0);
    }
    
    public void rotateClimberCW() {
    	climberRotator.set(1);
    }
    
    public void rotateWithJoystick(Joystick s) {
		double y = s.getAxis(AxisType.kY);
		climberRotator.set(-y);
	}
    public void extendWithJoystick(Joystick s) {
		double y = s.getAxis(AxisType.kY);
		climberExtender.set(-y);
	}
    
    public void rotateClimberCCW() {
    	climberRotator.set(-1);
    }
    
    public void stopClimberRotation() {
    	climberRotator.set(0);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

