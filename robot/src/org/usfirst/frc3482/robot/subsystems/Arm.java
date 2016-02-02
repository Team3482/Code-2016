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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Arm extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final SpeedController lowerJoint = RobotMap.armlowerJoint;
    private final Encoder lowerJointEncoder = RobotMap.armlowerJointEncoder;
    private final Encoder upperJointEncoder = RobotMap.armupperJointEncoder;
    private final SpeedController upperJoint = RobotMap.armupperJoint;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS


    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    
    public void spinLowerJointForward() {
    	lowerJoint.set(0.5);
    }
    
    public int getLowerJointEncoderPosition() {
    	return lowerJointEncoder.get();
    }
    
    public int getUpperJointEncoderPosition() {
    	return upperJointEncoder.get();
    }
    
    public void spinUpperJointBackward() {
    	upperJoint.set(-0.5);
    }
    
    public void spinLowerJointBackward() {
    	lowerJoint.set(-0.5);
    }
    
    public void stopUpperJoint() {
    	upperJoint.set(0.0);
    }
    
    public void stopLowerJoint() {
    	lowerJoint.set(0.0);
    } 
    
    public void spinTopToLocation(int desiredLoc, int delta) {
    	int initialLoc = upperJointEncoder.get();
    	if (initialLoc > desiredLoc) {
    		upperJoint.set(0.2);
    	} else if(initialLoc < desiredLoc){
    		upperJoint.set(-0.2);
    	}
    	while(true) {
    		int currentLoc = upperJointEncoder.get();
    		if (Math.abs(currentLoc-desiredLoc) <= delta) {
    			upperJoint.set(0.0);
    			break;
    		}
    	}
    }
    
    public void spinBottomToLocation(int desiredLoc, int delta) {
    	int initialLoc = lowerJointEncoder.get();
    	if (initialLoc > desiredLoc) {
    		lowerJoint.set(0.2);
    	} else if(initialLoc < desiredLoc){
    		lowerJoint.set(-0.2);
    	}
    	while(true) {
    		int currentLoc = lowerJointEncoder.get();
    		if (Math.abs(currentLoc-desiredLoc) <= delta) {
    			lowerJoint.set(0.0);
    			break;
    		}
    	}
    }
    
    public void runWithXboxController(Joystick s) {
		double leftY = s.getRawAxis(1);
		double rightY = s.getRawAxis(5);
		//sensitivity /= 100;
		double deadZone = 0.1;

//		if (leftY < deadZone && leftY > -deadZone) {
//			leftY = 0;
//		}
//		
		if (rightY < deadZone && rightY > -deadZone) {
			rightY = 0;
		}
		lowerJoint.set(leftY/3);
		upperJoint.set(rightY/3);
		//rightX *= sensitivity;
		//speedController1.set(leftY);
		//speedController2.set(rightY);
		/*if (leftY < -0.2) {
			Robot.clamp.extend();
			//Timer.delay(1);
		}*/
	}
    
} 

