package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class ArmPositionPortCullis extends Command {
    public ArmPositionPortCullis() {
        requires(Robot.chassis);
    	requires(Robot.arm);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.setTargetPortcullis();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.setTargetRest();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
