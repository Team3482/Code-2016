// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3482.robot.subsystems;

import org.usfirst.frc3482.robot.RobotMap;
import org.usfirst.frc3482.robot.commands.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Intake extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final Encoder intakeEncoder = RobotMap.intakeintakeEncoder;
    private final SpeedController intake = RobotMap.intake;
    private final SpeedController wheels = RobotMap.wheels;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS


    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    
    public void runWheels() {
    	wheels.set(1.0);
    }
    
    public void spinIntake() {
    	intake.set(1.0);
    }
    
    public void stopWheels() {
    	wheels.set(0.0);
    }
    
    public void stopIntake() {
    	intake.set(0.0);
    }
    
    public void spinToLocation(int desiredLoc, int delta) {
    	int initialLoc = intakeEncoder.get();
    	if (initialLoc > desiredLoc) {
    		intake.set(0.2);
    	} else if(initialLoc < desiredLoc){
    		intake.set(-0.2);
    	}
    	while(true) {
    		int currentLoc = intakeEncoder.get();
    		if (Math.abs(currentLoc-desiredLoc) <= delta) {
    			intake.set(0.0);
    			break;
    		}
    	}
    }
}

