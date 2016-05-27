package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class SpinAndShoot extends Command {

	boolean finished = false;
	int loop = 0;
 
    public SpinAndShoot() {
        requires(Robot.chassis);
    	requires(Robot.shooter);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loop = 0;
    	finished = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	loop++;
    	Robot.shooter.run();
		if(loop > 60) {
			Robot.intake.startFeed();
			if(loop > 150) {
				finished = true;
			}
		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.stopFeed();
		Robot.shooter.stopSpin();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
