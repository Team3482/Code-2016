package org.usfirst.frc3482.robot.subsystems;

import org.usfirst.frc3482.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final CANTalon climber = RobotMap.climber;
    private final TalonSRX climberExtender = RobotMap.climberExtender;
    
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
    
    public void stopExtendClimber() {
    	climberExtender.set(0);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

