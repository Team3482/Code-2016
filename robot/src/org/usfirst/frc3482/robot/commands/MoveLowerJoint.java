package org.usfirst.frc3482.robot.commands;

import org.usfirst.frc3482.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveLowerJoint extends Command {

	boolean finished = false;
	double speed;
	
	public MoveLowerJoint(double speed) {
    	this.speed = speed;
    }
	
    public MoveLowerJoint() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	speed = 1.0;
    }
   
    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("Execute");
    	Robot.arm.spinLowerJointAtSpeed(speed);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Stopped");
    	Robot.arm.stopLowerJoint();
//    	speed = 0;
    	//finished = true;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
