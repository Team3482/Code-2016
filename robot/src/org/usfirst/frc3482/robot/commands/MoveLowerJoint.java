package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class MoveLowerJoint extends Command {

	boolean direction = true;
	
    public MoveLowerJoint() {
        requires(Robot.chassis);
    	requires(Robot.arm);
    }
    
    public MoveLowerJoint(boolean direction) {
    	this.direction = direction;
    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.arm.stopLowerPID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.arm.runLowerJointWithXboxController(Robot.oi.getJoystick());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.arm.stopLowerJoint();
    	Robot.arm.startLowerPID();
    	//Robot.arm.stopLowerJoint();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
