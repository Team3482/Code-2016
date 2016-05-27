package org.usfirst.frc3482.robot.commands;

import org.usfirst.frc3482.robot.Robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoShoot extends Command {
	
	boolean finished;
	int loop;
	
    public AutoShoot() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.chassis);
    	requires(Robot.shooter);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	loop = 0;
    	finished = false;
    	System.out.println("SHOOOOOOOOT: "+".65 + "+Robot.oi.getJoystick().getAxis(Joystick.AxisType.kThrottle)*.05+" = "+.65+Robot.oi.getJoystick().getAxis(Joystick.AxisType.kThrottle)*.05);
    }

    // Called repeatedly when this Command is scheduled to run
    
    protected void execute() {
    	Robot.chassis.maintainDistanceVoltage(.65, 0, 0.015, true); //0.65, 0.015    then .72     then .69
    	loop++;
    	if(loop >= 60-60) {
    		System.out.println(loop);
    		Robot.shooter.run();
    		if(loop >= 125-60) {
    			Robot.intake.startFeed();
    			if(loop >= 150-60) {
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
