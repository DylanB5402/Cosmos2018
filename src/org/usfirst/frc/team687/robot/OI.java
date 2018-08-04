/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot;

import org.usfirst.frc.team687.robot.commands.arm.ClawForward;
import org.usfirst.frc.team687.robot.commands.arm.ClawReverse;
import org.usfirst.frc.team687.robot.commands.arm.ResetArmEncoder;
import org.usfirst.frc.team687.robot.commands.arm.SetArmPercentOutput;
import org.usfirst.frc.team687.robot.commands.arm.SetArmPosition;
import org.usfirst.frc.team687.robot.commands.drive.ResetDriveSensors;
import org.usfirst.frc.team687.robot.commands.drive.auto.DriveDistancePID;
import org.usfirst.frc.team687.robot.commands.drive.auto.DriveToXY;
import org.usfirst.frc.team687.robot.commands.drive.auto.TurnToAngle;
import org.usfirst.frc.team687.robot.constants.ArmConstants;
import org.usfirst.frc.team687.robot.utilities.NerdyMath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public Joystick leftStick;
	public Joystick rightStick;
	
	public OI() {
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		SmartDashboard.putData("Reset Drive Sensors", new ResetDriveSensors());
		SmartDashboard.putData("Reset Arm Encoder", new ResetArmEncoder());
//		SmartDashboard.putData("Arm positive Voltage", new SetArmPercentOutput(0.5));
//		SmartDashboard.putData("Arm negative Voltage", new SetArmPercentOutput(-0.5));
//		SmartDashboard.putData("Arm Score", new SetArmPosition(ArmConstants.kArmScorePos));
//		SmartDashboard.putData("claw forward", new ClawForward());
//		SmartDashboard.putData("claw reverse", new ClawReverse());
		SmartDashboard.putData("90 deg", new TurnToAngle(90, 1, 5));
		SmartDashboard.putData("Drive 100 in", new DriveDistancePID(NerdyMath.inchesToTicks(100)));
//		SmartDashboard.putData("Drive to (0, 50000)", new DriveToXY(0, 50000, 0.2, true));
		SmartDashboard.putData("Drive to (100000, 100000)", new DriveToXY(50000, 50000, 0.2, 0.00, true));

	}
	
	public double getLeftY() {
		return -leftStick.getY();
	}
	
	public double getLeftX() {
		return leftStick.getX();
	}
	
	public double getRightY() {
		return -rightStick.getY();
	}
	
	public double getRightX() {
		return rightStick.getX();
	}
	
	public boolean isLeftTriggerPulled() {
		return leftStick.getRawButton(1);
	}
	
	public void reportToSmartDashboard() {
//		SmartDashboard.putNumber("Left Stick Y", getLeftY());
//		SmartDashboard.putNumber("Right Stick Y", getRightY());
	}
}
