package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class Move extends Command {
	
	private double forwardY = 0;
	private double forwardX = 0;
	private double turn = 0;
	private double time = 0;

	public Move(/*double forwardY, double forwardX, double turn, double time*/) {
    	requires(Robot.chassis);
    	
    	/*this.forwardY = forwardY;
    	this.forwardX = forwardX;
		this.turn = turn;
		this.time = time;*/
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//disabled safety and moves to a location
    			/*Robot.chassis.setSafety(false);
    			Robot.chassis.move(forwardY, forwardX, turn);
    			//waits for a given time
    			Timer.delay(time);
    			//enables safety and stops
    			Robot.chassis.move(0.0, 0.0, 0.0);
    			Robot.chassis.setSafety(true);*/
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
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
