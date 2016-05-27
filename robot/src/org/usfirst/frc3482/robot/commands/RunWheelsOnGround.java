package org.usfirst.frc3482.robot.commands;

import org.usfirst.frc3482.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunWheelsOnGround extends Command {

    public RunWheelsOnGround() {
    	requires(Robot.chassis);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.stopPID();
    	Robot.intake.runWheels();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("Running");
    	Robot.intake.runWithXboxController(Robot.oi.getxboxController());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.stopWheels();
    	Robot.intake.stopIntake();
    	Robot.intake.startPID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
