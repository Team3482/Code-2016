package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class AutonomousCommand extends CommandGroup {

    public AutonomousCommand() {
        requires(Robot.chassis);
        //addSequential(new Move(-0.6, 0.0, 0.1), 3.0);
//        Timer.delay(1.0);
//        addSequential(new Move(0.0, 1.0, 0.1), 0.18);
        addSequential(new Rotate(90, false));
        
    }

    /*// Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Moving");
    	//Robot.chassis.setDesiredDistance(12);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.chassis.moveDistance();
    	Robot.chassis.move(0.4, 0.0);
    	Timer.delay(5.0);
    	Robot.chassis.move(0.0, 0.5);
    	Timer.delay(0.5);
    	Robot.chassis.move(0.0, 0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //return Robot.chassis.shouldBeMoving();
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.chassis.disableRotation();
    	//Robot.chassis.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }*/
}
