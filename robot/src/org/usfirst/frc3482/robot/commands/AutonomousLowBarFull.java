package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class AutonomousLowBarFull extends CommandGroup {
	
    public AutonomousLowBarFull() {
        requires(Robot.chassis);
        requires(Robot.arm);
        requires(Robot.intake);
        requires(Robot.camera);

        System.out.println("LOWWWWWW FULLLLLL");
        addSequential(new LowerTargetIntake());
        addSequential(new Move(.7, 0, 268), 4.0);
    	addSequential(new Move(.7, 60, 0), 2.0);
    	addSequential(new Move(.7, 0, 138), 2.0);
    	addSequential(new SpinAndShoot(), 5.0);
    }
    
}
