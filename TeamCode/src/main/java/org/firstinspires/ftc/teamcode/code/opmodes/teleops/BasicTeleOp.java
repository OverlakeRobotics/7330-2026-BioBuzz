package org.firstinspires.ftc.teamcode.code.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.helpers.AutoPoseHandoff;
import org.firstinspires.ftc.teamcode.code.parts.*;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


@Config
@TeleOp(name="Basic TeleOp", group="TeleOp")

public class BasicTeleOp extends OpMode {
    private OdometryHolonomicDrivetrain driveTrain;
    public static double driveVelocity = 2800;

    private Shooter pollenShooter;
    //private Shooter nectarShooter;

    public static double shooterVelocity = 0;
    public static double TICKS_PER_SEC = (double) 28 /60;

    private static Pose2D startPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    private Pusher pusher;
    private Intake intake;
    private static double intakePower;
    private boolean shooterEnabled = false;
    private boolean reverseShooterEnabled = false;
    private boolean intakeEnabled = false;


    @Override
    public void init() {
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"))
        );

        Pose2D autoEndPos = AutoPoseHandoff.consume();
        driveTrain.setPosition(autoEndPos != null ? autoEndPos : startPos);

        pollenShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"));
        //nectarShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "nectarShooter"));

        pusher = new Pusher(hardwareMap.get(Servo.class, "pusher"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
    }

    @Override
    public void loop() {
        boolean shooterTogglePressed = gamepad1.yWasPressed();
        boolean intakeTogglePressed = gamepad1.bWasPressed();
        boolean shooterSlowerPressed = gamepad1.rightBumperWasPressed();
        boolean shooterFasterPressed = gamepad1.leftBumperWasPressed();
        boolean pusherPressed = gamepad1.rightTriggerWasPressed();
        boolean shooterReverseTogglePressed = gamepad1.aWasPressed();

        driveTrain.updatePosition();
        Pose2D currentPos = driveTrain.getPosition();

        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * driveVelocity);
        driveTrain.drive();

        if (shooterTogglePressed) {
            shooterEnabled = !shooterEnabled;
            reverseShooterEnabled = false;
            if (shooterEnabled) {
                shooterVelocity = 3000 * TICKS_PER_SEC;
            }
        }
        if (shooterReverseTogglePressed) {
            reverseShooterEnabled = !reverseShooterEnabled;
            shooterEnabled = false;
            if (reverseShooterEnabled) {
                shooterVelocity = -200 * TICKS_PER_SEC;
            }
        }
        if (intakeTogglePressed) {
            intakeEnabled = !intakeEnabled;
            intakePower = intakeEnabled ? 0.5 : 0;
        }

        if (shooterSlowerPressed) {
            shooterVelocity -= 100 * TICKS_PER_SEC;
        } else if (shooterFasterPressed) {
            shooterVelocity += 100 * TICKS_PER_SEC;
        }

        pusher.update(pusherPressed);
        intake.setIntakePower(intakePower);

        pollenShooter.setShooterVelocity(shooterEnabled ? shooterVelocity : 0);
        //pollenShooter.setShooterPower(shooterPower);

        telemetry.addLine("shooter: y | right/left bumper");
        telemetry.addLine("intake: b");
        telemetry.addLine("pusher: right trigger");
        telemetry.addLine();

        telemetry.addData("target rpm: ", shooterVelocity * 60 / 28);
        telemetry.addData("actual rpm: ", pollenShooter.getShooterVelocity() * 60 / 28);
        telemetry.addData("intake:", intakeEnabled ? "on" : "off");
        telemetry.update();
    }

}
