package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class Shoot extends Command {

	boolean finished = false;
	
    public Shoot() {
    	requires(Robot.chassis);
    	requires(Robot.shooter);
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intake.runWheels();
    	Timer.delay(0.2);
    	Robot.intake.stopWheels();
    	finished = true;
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
    }
}
