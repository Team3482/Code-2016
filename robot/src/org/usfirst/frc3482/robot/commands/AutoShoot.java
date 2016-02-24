package org.usfirst.frc3482.robot.commands;

import org.usfirst.frc3482.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoShoot extends Command {
	
	boolean finished = false;

    public AutoShoot() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.chassis);
    	requires(Robot.shooter);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    int loop = 0;
    protected void execute() {
    	Robot.chassis.maintainDistanceVoltage(.37, .01, 0.0);
    	loop++;
    	if(loop >= 60) {
    		System.out.println(loop);
    		Robot.shooter.spin();
    		if(loop >= 125) {
    			Robot.intake.startFeed();
    			if(loop >= 150) {
    				Robot.intake.stopFeed();
    				Robot.shooter.stopSpin();
    				finished = true;
    			}
    		}
    	}
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
