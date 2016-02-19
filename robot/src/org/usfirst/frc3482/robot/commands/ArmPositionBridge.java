package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class ArmPositionBridge extends Command {

    public ArmPositionBridge() {
        requires(Robot.chassis);
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//Robot.arm.setTargetLower();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.intake.runWheels();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.arm.setTargetRest();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
