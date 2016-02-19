package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class MoveUpperJoint extends Command {

    public MoveUpperJoint() {
        requires(Robot.chassis);
    	requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.arm.stopUpperPID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.arm.runUpperJointWithXboxController(Robot.oi.getJoystick());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.arm.stopUpperJoint();
    	Robot.arm.startUpperPID();
    	//Robot.arm.stopUpperJoint();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
