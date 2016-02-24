package org.usfirst.frc3482.robot.commands;

import org.usfirst.frc3482.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Wait extends Command {

	double loops;
	boolean finished = false;
    public Wait(double loops) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.chassis);  
    	requires(Robot.arm);
    	requires(Robot.intake);
    	this.loops = loops;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    int i =0;
    protected void execute() {
    	if(i >= loops) {
    		finished = true;
    	}
    	Robot.intake.maintainPosition();
    	Robot.arm.maintainLowerJointPosition();
    	Robot.arm.maintainUpperJointPosition();
    	i++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
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
