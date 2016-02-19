package org.usfirst.frc3482.robot.commands;

import org.usfirst.frc3482.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Rotate extends Command {

	private double rotateValue;
	private boolean isRelative;
	
    public Rotate(double rotateValue, boolean isRelative) {
        requires(Robot.chassis);
        
        this.rotateValue = rotateValue;
        this.isRelative = isRelative;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(isRelative) {
    		Robot.chassis.rotateByAngle(rotateValue);
    	} else {
    		Robot.chassis.rotateToAngle(rotateValue);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
