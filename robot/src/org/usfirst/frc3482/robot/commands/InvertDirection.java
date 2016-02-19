package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class InvertDirection extends Command {

	boolean finished = false;
 
    public InvertDirection() {
    	requires(Robot.chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.chassis.invertMotors();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//finished = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
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
