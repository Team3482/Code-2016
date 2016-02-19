package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class MoveIntake extends Command {
	
    public MoveIntake() {
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
    	Robot.intake.runWithXboxController(Robot.oi.getJoystick());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.stopWheels();
    	Robot.intake.startPID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
