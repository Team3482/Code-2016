package org.usfirst.frc3482.robot.commands;

import org.usfirst.frc3482.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Rotate extends Command {

	private double rotateValue;
	private boolean isRelative;
	private int loops;
	private boolean finished;
	
	//Rotates the robot to a given degrees (-180 to 180)
    public Rotate(double rotateValue, boolean isRelative) {
        requires(Robot.chassis);
        this.rotateValue = rotateValue;
        this.isRelative = isRelative;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loops = 0;
    	finished = false;
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("Rotating");
    	if(loops <= 150) {
	    	if(isRelative) {
	    		Robot.chassis.rotateByAngle(rotateValue);
	    	} else {
	    		Robot.chassis.rotateToAngle(rotateValue);
	    	}
    	} else {
    		finished = true;
    	}
    	loops++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.chassis.disableRotation();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
