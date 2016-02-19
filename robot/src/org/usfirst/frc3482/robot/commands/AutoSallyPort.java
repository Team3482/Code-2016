package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class AutoSallyPort extends Command {

    public AutoSallyPort() {
        requires(Robot.chassis);
    	requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.arm.setUpperRest();
    	Timer.delay(.5);
    	Robot.arm.setLowerSally();
    	Robot.arm.setUpperSally();
    	Timer.delay(.5);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.arm.setLowerRest();
    	Robot.arm.setUpperHome();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
