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
@TeleOp(name="Odemetry TeleOp", group="TeleOp")

public class OdometryTeleOp extends OpMode {
    private OdometryHolonomicDrivetrain driveTrain;
    public static double driveVelocity = 2800;

    private Shooter pollenShooter;
    //private Shooter nectarShooter;

    public static double shooterVelocity = 0;
    public static double TICKS_PER_SEC = (double) 28 /60;

    private static Pose2D startPos = new Pose2D(DistanceUnit.INCH, -63, 63, AngleUnit.DEGREES, 0);
    private Pose2D[] presets = {
            new Pose2D(DistanceUnit.INCH, -63, 15, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 63, 15, AngleUnit.DEGREES, 180),
            new Pose2D(DistanceUnit.INCH, 21, 63, AngleUnit.DEGREES, -90)
    };

    private Pusher pusher;
    private Intake intake;
    private static double intakePower;
    private boolean shooterEnabled = false;
    private boolean reverseShooterEnabled = false;
    private boolean intakeEnabled = false;
    private int poseIndex = 0;
    private boolean drivingToPose = false;

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
        driveTrain.updatePosition();
        Pose2D currentPos = driveTrain.getPosition();
        //telemetry.addData("Current Position: ", currentPos);

        if (!drivingToPose) {
            if (gamepad1.dpadLeftWasPressed()) {
                poseIndex = Math.max(0, poseIndex - 1);
            } else if (gamepad1.dpadRightWasPressed()) {
                poseIndex = Math.min(presets.length - 1, poseIndex + 1);
            }

            if (gamepad1.xWasPressed()) {
                driveTrain.setVelocity((int) driveVelocity);
                driveTrain.setPositionDrive(presets[poseIndex]);
                drivingToPose = true;
            }
        }

        boolean manualOverride = Math.abs(gamepad1.left_stick_x) > 0.1
                || Math.abs(gamepad1.left_stick_y) > 0.1
                || Math.abs(gamepad1.right_stick_x) > 0.1;
        if (drivingToPose && manualOverride) {
            driveTrain.stop();
            drivingToPose = false;
        }

        if (!drivingToPose) {
            driveTrain.setVelocityDrive(
                    -gamepad1.left_stick_y * driveVelocity,
                    -gamepad1.left_stick_x * driveVelocity,
                    -gamepad1.right_stick_x * driveVelocity);
        }
        driveTrain.drive();

        if (drivingToPose && !driveTrain.isDriving()) {
            driveTrain.stop();
            drivingToPose = false;
        }

        if (gamepad1.yWasPressed()) {
            shooterEnabled = !shooterEnabled;
            reverseShooterEnabled = false;
            if (shooterEnabled) {
                shooterVelocity = 3000 * TICKS_PER_SEC;
            }
        }
        if (gamepad1.aWasPressed()) {
            reverseShooterEnabled = !reverseShooterEnabled;
            shooterEnabled = false;
            if (reverseShooterEnabled) {
                shooterVelocity = -200 * TICKS_PER_SEC;
            }
        }
        if (gamepad1.bWasPressed()) {
            intakeEnabled = !intakeEnabled;
            intakePower = intakeEnabled ? 0.5 : 0;
        }

        if (gamepad1.rightBumperWasPressed()) {
            shooterVelocity -= 100 * TICKS_PER_SEC;
        } else if (gamepad1.leftBumperWasPressed()) {
            shooterVelocity += 100 * TICKS_PER_SEC;
        }

        pusher.update(gamepad1.rightTriggerWasPressed());
        intake.setIntakePower(intakePower);

        pollenShooter.setShooterVelocity(shooterEnabled ? shooterVelocity : 0);
        //pollenShooter.setShooterPower(shooterPower);



        telemetry.addLine("D-pad left/right: select pose | X: drive to pose | sticks: manual drive/cancel");
        telemetry.addLine("shooter: y | right/left bumper");
        telemetry.addLine("intake: b");
        telemetry.addLine("pusher: right trigger");
        telemetry.addLine();

        telemetry.addData("selected pose", "%d / %d", poseIndex + 1, presets.length);
        telemetry.addData("target pose", "X: %.1f, Y: %.1f, H: %.1f",
                presets[poseIndex].getX(DistanceUnit.INCH),
                presets[poseIndex].getY(DistanceUnit.INCH),
                presets[poseIndex].getHeading(AngleUnit.DEGREES));
        telemetry.addData("driving to pose", drivingToPose);
        telemetry.addData("current pose", "X: %.1f, Y: %.1f, H: %.1f",
                currentPos.getX(DistanceUnit.INCH),
                currentPos.getY(DistanceUnit.INCH),
                currentPos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("target rpm: ", shooterVelocity * 60 / 28);
        telemetry.addData("actual rpm: ", pollenShooter.getShooterVelocity() * 60 / 28);
        telemetry.addData("intake:", intakeEnabled ? "on" : "off");
        telemetry.update();
    }
}
