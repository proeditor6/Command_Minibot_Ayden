package frc.robot.subsystems;

import javax.swing.text.StyledEditorKit.AlignmentAction;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Drive extends SubsystemBase {

    private static final double positionConversionFactor = 12.0 / 72.0 * Math.PI * Units.inchesToMeters(4);
    private static final double velocityConversionFactor = positionConversionFactor / 60;

    private final CANSparkMax leftMotor = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(3, MotorType.kBrushless);
    public final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final AHRS gyro = new AHRS();
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(12.25));
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);

    private final PIDController rightPID = new PIDController(1.0, 0.0, 0.0);
    private final PIDController leftPID = new PIDController(1.0, 0.0, 0.0);

    private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0.0, 1.0);
    private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0.0, 1.0);


    private DifferentialDriveWheelPositions getWheelPositions()
    {
        return new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    private DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    private void applyChassisSpeeds (ChassisSpeeds speeds)
    {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        
        leftMotor.setVoltage(
            leftPID.calculate(leftEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond) +
            leftFF.calculate(wheelSpeeds.leftMetersPerSecond)
        );

        rightMotor.setVoltage(
            rightPID.calculate(rightEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond) +
            rightFF.calculate(wheelSpeeds.rightMetersPerSecond)
        );
    }

    public Drive() {
        leftMotor.setInverted(false);
        rightMotor.setInverted(false);
        drive.setDeadband(0.1);
        drive.setMaxOutput(1);
        
        leftEncoder.setPositionConversionFactor(positionConversionFactor);
        leftEncoder.setVelocityConversionFactor(velocityConversionFactor);

        rightEncoder.setPositionConversionFactor(positionConversionFactor);
        rightEncoder.setVelocityConversionFactor(velocityConversionFactor);

        AutoBuilder.configureLTV(
            odometry::getPoseMeters, 
            pose -> odometry.resetPosition(gyro.getRotation2d(), getWheelPositions(), pose), 
            () -> kinematics.toChassisSpeeds(getWheelSpeeds()), 
            this::applyChassisSpeeds,
            0.02, 
            new ReplanningConfig(),
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == Alliance.Red : false;
            },
            this
        );
    }

    public void periodic() {
        SmartDashboard.putNumber("Left Drive Velocity (mps)", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Drive Velocity (mps)", rightEncoder.getVelocity());

        SmartDashboard.putNumber("Left Drive Distance (m)", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Drive Distance (m)", rightEncoder.getPosition());

        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public Command getTeleopDriveCommand() {
        return run(() -> {
            double speed = 1.0-Math.pow((RobotContainer.controller.getRightTriggerAxis()), 2.0);

            RobotContainer.drive.drive.arcadeDrive(
                -RobotContainer.controller.getLeftY() * speed, 
                -RobotContainer.controller.getLeftX() * speed
            );
        });
    }
}
