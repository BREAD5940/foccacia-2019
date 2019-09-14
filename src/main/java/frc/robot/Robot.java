/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import kotlin.ranges.RangesKt;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  Joystick joy = new Joystick(0);

	public double getForwardAxis() {
		return -1 * joy.getRawAxis(xboxmap.Axis.LEFT_JOYSTICK_Y);
	}

	public double getTurnAxis() {
		return joy.getRawAxis(4);
	}

  private WPI_TalonSRX mLeft, mRight, sLeft, sRight;
  private SpeedControllerGroup left, right;
  private DifferentialDrive drive;
  
  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";
  // private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    mLeft = new WPI_TalonSRX(4);
    sLeft = new WPI_TalonSRX(3);

    mRight = new WPI_TalonSRX(1);
    sRight = new WPI_TalonSRX(2);

    mLeft.setInverted(true);
    sLeft.setInverted(true);

    // sLeft.setInverted(InvertType.OpposeMaster);
    // sLeft.set(ControlMode.Follower, mRight.getDeviceID());
    // sRight.set(ControlMode.Follower, mRight.getDeviceID());

    // sLeft.set(ControlMode.PercentOutput, 0);
    // sRight.set(ControlMode.PercentOutput, 0);

    // drive = new DifferentialDrive(mLeft, sLeft);

    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // drive.curvatureDrive(getForwardAxis(), getTurnAxis(), joy.getRawButton(xboxmap.Buttons.RIGHT_JOYSTICK_BUTTON));
    // mLeft.set(ControlMode.PercentOutput, 0.5);
    // mRight.set(ControlMode.PercentOutput, 0.5);

    double forward = getForwardAxis();
    double turn = getTurnAxis();

    double left = forward + turn;
    double right = forward - turn;

    mLeft.set(ControlMode.PercentOutput, left);
    mRight.set(ControlMode.PercentOutput, right);
    sLeft.set(ControlMode.PercentOutput, left);
    sRight.set(ControlMode.PercentOutput, right);

	// joy.setRumble(RumbleType.kLeftRumble, 1);


  }



	public void arcadeDrive(double linear, double rotation) {
		arcadeDrive(linear, rotation, true);
	}

	public void arcadeDrive(double linearPercent, double rotationPercent, boolean squareInputs) {
		linearPercent = Util.limit(linearPercent, 1);
		linearPercent = Util.deadband(linearPercent, 0.02);

		rotationPercent = Util.limit(rotationPercent, 1);
		rotationPercent = Util.deadband(rotationPercent, 0.02);

		// Square the inputs (while preserving the sign) to increase fine control
		// while permitting full power.
		if (squareInputs) {
			linearPercent = Math.copySign(linearPercent * linearPercent, linearPercent);
			rotationPercent = Math.copySign(rotationPercent * rotationPercent, rotationPercent);
		}

		double leftMotorOutput;
		double rightMotorOutput;

		double maxInput = Math.copySign(Math.max(Math.abs(linearPercent), Math.abs(rotationPercent)), linearPercent);

		if (linearPercent >= 0.0) {
			// First quadrant, else second quadrant
			if (rotationPercent >= 0.0) {
				leftMotorOutput = maxInput;
				rightMotorOutput = linearPercent - rotationPercent;
			} else {
				leftMotorOutput = linearPercent + rotationPercent;
				rightMotorOutput = maxInput;
			}
		} else {
			// Third quadrant, else fourth quadrant
			if (rotationPercent >= 0.0) {
				leftMotorOutput = linearPercent + rotationPercent;
				rightMotorOutput = maxInput;
			} else {
				leftMotorOutput = maxInput;
				rightMotorOutput = linearPercent - rotationPercent;
			}
		}
		// Logger.log("Linear input " + linearPercent + " turn input " +
		// rotationPercent);
		// Logger.log("left motor output " + leftMotorOutput + " right motor output " +
		// rightMotorOutput);

		// ChassisState mTarget = new ChassisState(linearPercent * 6, -1 * rotationPercent * 6);

		// WheelState mCalced = getDifferentialDrive().solveInverseKinematics(mTarget);

		// double left = mCalced.get(true);

		// double right = mCalced.get(false);

		// tankDrive(left/12, right/12);

		tankDrive(leftMotorOutput, rightMotorOutput);
		// tankDrive(0.2, 0.2);
	}

	public void curvatureDrive(double linearPercent, double curvaturePercent, boolean isQuickTurn) {
		double angularPower;
		boolean overPower;

		if (isQuickTurn) {
			if (Math.abs(linearPercent) < kQuickStopThreshold) {
				quickStopAccumulator = (1 - kQuickStopAlpha) * quickStopAccumulator
						+ kQuickStopAlpha * RangesKt.coerceIn(curvaturePercent, -1, 1) * 2.0;
			}

			overPower = true;
			angularPower = curvaturePercent;
		} else {
			overPower = false;
			angularPower = Math.abs(linearPercent) * curvaturePercent - quickStopAccumulator;

			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0;
			}
		}

		double leftMotorOutput = linearPercent + angularPower;
		double rightMotorOutput = linearPercent - angularPower;

		// If rotation is overpowered, reduce both outputs to within acceptable range
		if (overPower) {
			if (leftMotorOutput > 1.0) {
				rightMotorOutput -= leftMotorOutput - 1.0;
				leftMotorOutput = 1.0;
			} else if (rightMotorOutput > 1.0) {
				leftMotorOutput -= rightMotorOutput - 1.0;
				rightMotorOutput = 1.0;
			} else if (leftMotorOutput < -1.0) {
				rightMotorOutput -= leftMotorOutput + 1.0;
				leftMotorOutput = -1.0;
			} else if (rightMotorOutput < -1.0) {
				leftMotorOutput -= rightMotorOutput + 1.0;
				rightMotorOutput = -1.0;
			}
		}

		// Normalize the wheel speeds
		double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
		if (maxMagnitude > 1.0) {
			leftMotorOutput /= maxMagnitude;
			rightMotorOutput /= maxMagnitude;
		}

		tankDrive(leftMotorOutput, rightMotorOutput);
	}

	public void tankDrive(double leftPercent, double rightPercent) {
		// lastCommandedVoltages = Arrays.asList(leftPercent * 12, rightPercent * 12);

		// Logger.log("Left error: " + getLeft().getClosedLoopError().getFeet());
		// Logger.log("Right error: " + getRight().getClosedLoopError().getFeet());

		// Logger.log("Left speed: " +
		// getLeft().getMaster().getSensorVelocity().getValue());
		// Logger.log("Right speed: " + getRight().getVelocity().getValue());

		if (leftPercent < 0.06 && leftPercent > -0.06)
			leftPercent = 0.0;

		if (rightPercent < 0.06 && rightPercent > -0.06)
			rightPercent = 0.0;

		mLeft.set(ControlMode.PercentOutput, leftPercent); // because C O M P E N S A T I O N
		mRight.set(ControlMode.PercentOutput, rightPercent);

		// 2.1 meters per second in low gear
		//

		// getLeft().getMaster().set(ControlMode.Velocity, LengthKt.getFeet(leftPercent
		// * 8)); // because C O M P E N S A T I O N
		// getRight().getMaster().set(ControlMode.Velocity,
		// LengthKt.getFeet(rightPercent * 8));
  }
  
  private static double kQuickStopThreshold = 0.2;// DifferentialDrive.kDefaultQuickStopThreshold;
	private static double kQuickStopAlpha = 0.1;// DifferentialDrive.kDefaultQuickStopAlpha;
	private double quickStopAccumulator = 0;


}
