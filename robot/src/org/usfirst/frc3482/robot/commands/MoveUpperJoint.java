package org.usfirst.frc3482.robot.commands;

import org.usfirst.frc3482.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveUpperJoint extends Command {

	boolean finished = false;
	double speed;
	
    public MoveUpperJoint() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	speed = 1.0;
    }
    
    public MoveUpperJoint(double speed) {
    	this.speed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("Execute");
    	Robot.arm.spinUpperJointAtSpeed(speed);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Stopped");
//    	finished = true;
//    	speed = 0.0;
    	Robot.arm.stopUpperJoint();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
