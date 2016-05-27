package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class AutonomousGunIt extends CommandGroup {
	
    public AutonomousGunIt() {
        requires(Robot.chassis);
        requires(Robot.arm);
        requires(Robot.intake);
        requires(Robot.camera);
        
		System.out.println("GUNNNNNN");
		addSequential(new Move(1, 0, -1000), 2);
		addSequential(new Move(.5, 0, -1000), 2);
    }

}
