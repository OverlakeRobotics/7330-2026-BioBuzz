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
    protected OdometryHolonomicDrivetrain driveTrain;
    public static double driveVelocity = 2800;

    private Shooter pollenShooter;
    private Shooter nectarShooter;

    public static double pollenShooterVelocity = 0;
    public static double nectarShooterVelocity = 0;
    public static double TICKS_PER_SEC = (double) 28 /60;

    private static Pose2D startPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    private Pusher pusher;
    private Intake intake;
    private static double intakePower;
    private boolean pollenEnabled = false;
    private boolean reversePollenEnabled = false;
    private boolean nectarEnabled = false;
    private boolean intakeEnabled = false;

    private final double pollenP = 1.5;
    private final double pollenI = 0.0;
    private final double pollenD = 0.0;
    private final double pollenF = 12.0;
    private final double nectarP = 1.5;
    private final double nectarI = 0.0;
    private final double nectarD = 0.0;
    private final double nectarF = 12.0;


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
        driveTrain.setPosition(autoEndPos != null ? autoEndPos : getDefaultStartPosition());

        pollenShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"), DcMotorEx.Direction.REVERSE);
        nectarShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "nectarShooter"), DcMotorEx.Direction.FORWARD);
        pollenShooter.setShooterPIDFCoefficients(pollenP, pollenI, pollenD, pollenF);
        nectarShooter.setShooterPIDFCoefficients(nectarP, nectarI, nectarD, nectarF);
        pusher = new Pusher(hardwareMap.get(Servo.class, "pusher"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
    }

    @Override
    public void loop() {
        updateDrive();
        updateMechanisms();
        addDriveTelemetry();
        addMechanismTelemetry();
        telemetry.update();
    }

    protected Pose2D getDefaultStartPosition() {
        return startPos;
    }

    protected void updateDrive() {
        driveTrain.updatePosition();
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * driveVelocity);
        driveTrain.drive();
    }

    protected void updateMechanisms() {
        boolean pollenTogglePressed = gamepad1.yWasPressed();
        boolean intakeTogglePressed = gamepad1.bWasPressed();
        boolean pollenReverseTogglePressed = gamepad1.aWasPressed();
        boolean nectarTogglePressed = isNectarTogglePressed();
        boolean pusherPressed = gamepad1.rightTriggerWasPressed();

        if (pollenTogglePressed) {
            pollenEnabled = !pollenEnabled;
            reversePollenEnabled = false;
            if (pollenEnabled) {
                pollenShooterVelocity = 3000 * TICKS_PER_SEC;
            }
        }
        if (pollenReverseTogglePressed) {
            reversePollenEnabled = !reversePollenEnabled;
            pollenEnabled = false;
            if (reversePollenEnabled) {
                pollenShooterVelocity = -200 * TICKS_PER_SEC;
            }
        }
        if (intakeTogglePressed) {
            intakeEnabled = !intakeEnabled;
            intakePower = intakeEnabled ? 0.5 : 0;
        }
        if (nectarTogglePressed) {
            nectarEnabled = !nectarEnabled;
            nectarShooterVelocity = nectarEnabled ? 4100 * TICKS_PER_SEC : 0;
        }

        updateAdditionalShooterControls();

        pusher.update(pusherPressed);
        intake.setIntakePower(intakePower);

        nectarShooter.setShooterVelocity(nectarEnabled ? nectarShooterVelocity : 0);
        pollenShooter.setShooterVelocity(pollenEnabled ? pollenShooterVelocity : 0);
    }

    protected boolean isNectarTogglePressed() {
        return gamepad1.xWasPressed();
    }

    protected String getNectarToggleControlName() {
        return "x";
    }

    protected void updateAdditionalShooterControls() {
        // Subclasses can add shooter controls without duplicating mechanism setup.
    }

    protected void addDriveTelemetry() {
        // Subclasses can report drive-specific status here.
    }

    protected void addMechanismTelemetry() {
        telemetry.addLine("pollen shooter: y");
        telemetry.addLine("reverse pollen shooter: a");
        telemetry.addLine("nectar shooter: " + getNectarToggleControlName());
        telemetry.addLine("intake: b");
        telemetry.addLine("pusher: right trigger");
        telemetry.addLine();
        telemetry.addData("intake:", intakeEnabled ? "on" : "off");
        telemetry.addLine("----pollen----");
        telemetry.addData("target rpm: ", pollenShooterVelocity * 60 / 28);
        telemetry.addData("actual rpm: ", pollenShooter.getShooterVelocity() * 60 / 28);
        telemetry.addLine("----nectar----");
        telemetry.addData("target rpm: ", nectarShooterVelocity * 60 / 28);
        telemetry.addData("actual rpm: ", nectarShooter.getShooterVelocity() * 60 / 28);
    }

    @Override
    public void stop() {
        pollenShooter.setShooterVelocity(0);
        nectarShooter.setShooterVelocity(0);
        intake.setIntakePower(0);
        pusher.home();
    }

}
