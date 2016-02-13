package org.usfirst.frc3482.robot.commands;

import org.usfirst.frc3482.robot.Robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FeedShooter extends Command {

	boolean direction;
	public FeedShooter() {
		// Use requires() here to declare subsystem dependencies
		//very strange but does not work at same time (shooter and intake) if not commented out
		//requires(Robot.chassis);
		//requires(Robot.shooter);
		direction = true;
	}

	public FeedShooter(boolean direction) {
		//very strange but does not work at same time (shooter and intake) if not commented out
		//requires(Robot.chassis);
		//requires(Robot.shooter);
		this.direction = direction;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (direction) { 
			Robot.intake.startFeed();
			Robot.intake.runWheels();
		} else {
			Robot.intake.runWheelsBackward();
			Robot.intake.reverseFeed();
		}
			
		//Robot.oi.xboxController.getAxis(Joystick.AxisType.kTwist);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.intake.stopFeed();
		Robot.intake.stopWheels();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
