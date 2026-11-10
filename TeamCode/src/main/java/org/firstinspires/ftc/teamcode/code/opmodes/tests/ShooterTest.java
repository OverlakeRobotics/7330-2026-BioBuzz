package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.code.parts.*;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;


@Config
@TeleOp(name="Shooter Test", group="Test")

public class ShooterTest extends OpMode {
    private BasicHolonomicDrivetrain driveTrain;
    public static double driveVelocity = 2800;

    private Shooter pollenShooter;
    private Shooter nectarShooter;

    public static double pollenShooterVelocity = 0;
    public static double nectarShooterVelocity = 0;
    public static double TICKS_PER_SEC = (double) 28 /60;

    public static double SHOOTER_P = 1.5;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 12.0;
    public static double PUSHER_RPM_TOLERANCE = 50.0;
    public static double PUSHER_STABLE_TIME_MS = 250.0;

    private Pusher pusher;
    private boolean pollenWasAtTarget = false;
    private boolean pollenReady = false;
    private final ElapsedTime pollenAtTargetTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private boolean timing = false;
    private double lastRecoveryTime = -1;
    private boolean spinUpTiming = false;
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private double lastSpinUpTime = -1;

    private double appliedShooterP = Double.NaN;
    private double appliedShooterI = Double.NaN;
    private double appliedShooterD = Double.NaN;
    private double appliedShooterF = Double.NaN;


    @Override
    public void init() {
        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );

        pollenShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"), DcMotorEx.Direction.REVERSE);
        nectarShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "nectarShooter"), DcMotorEx.Direction.FORWARD);
        pusher = new Pusher(hardwareMap.get(Servo.class, "pusher"));
        updateShooterPIDF();

    }

    @Override
    public void loop() {
        updateShooterPIDF();

        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * driveVelocity);
        driveTrain.drive();

        if (gamepad1.yWasPressed()) {
            if (pollenShooterVelocity == 0) {
                spinUpTiming = true;
                spinUpTimer.reset();
            }
            pollenShooterVelocity = 3000 * TICKS_PER_SEC;
        } else if (gamepad1.xWasPressed()) {
            pollenShooterVelocity = 0;
        } else if (gamepad1.bWasPressed()) {
            nectarShooterVelocity = 4000 * TICKS_PER_SEC;
        } else if (gamepad1.aWasPressed()) {
            nectarShooterVelocity = 0;
        } else if (gamepad1.leftBumperWasPressed()) {
            pollenShooterVelocity -= 100 * TICKS_PER_SEC;
        } else if (gamepad1.rightBumperWasPressed()) {
            pollenShooterVelocity += 100 * TICKS_PER_SEC;
        } else if (gamepad1.rightTriggerWasPressed()) {
            nectarShooterVelocity += 100 * TICKS_PER_SEC;
        } else if (gamepad1.leftTriggerWasPressed()) {
            nectarShooterVelocity -= 100 * TICKS_PER_SEC;
        }

        if (gamepad2.dpadUpWasPressed()) {
            SHOOTER_P += 0.1;
        } else if (gamepad2.dpadDownWasPressed()) {
            SHOOTER_P = Math.max(0, SHOOTER_P - 0.1);
        } else if (gamepad2.dpadRightWasPressed()) {
            SHOOTER_I += 0.1;
        } else if (gamepad2.dpadLeftWasPressed()) {
            SHOOTER_I = Math.max(0, SHOOTER_I - 0.01);
        } else if (gamepad2.yWasPressed()) {
            SHOOTER_D += 0.1;
        } else if (gamepad2.aWasPressed()) {
            SHOOTER_D = Math.max(0, SHOOTER_D - 0.01);
        } else if (gamepad2.xWasPressed()) {
            SHOOTER_F += 0.5;
        } else if (gamepad2.bWasPressed()) {
            SHOOTER_F = Math.max(0, SHOOTER_F - 0.1);
        }

        pollenShooter.setShooterVelocity(pollenShooterVelocity);
        nectarShooter.setShooterVelocity(nectarShooterVelocity);

        boolean pollenAtSpeed = isPollenAtTargetSpeed();
        boolean shotStarted = gamepad1.startWasPressed()
                && pollenAtSpeed
                && pusher.getState() == 0;
        pusher.update(shotStarted);

        if (shotStarted) {
            recoveryTimer.reset();
            timing = true;
            pollenWasAtTarget = false;
            pollenReady = false;
        }
        if (timing && pollenReady) {
            lastRecoveryTime = recoveryTimer.milliseconds();
            timing = false;
        }
        if (spinUpTiming && pollenAtSpeed) {
            lastSpinUpTime = spinUpTimer.milliseconds();
            spinUpTiming = false;
        }

        telemetry.addData("last recovery ms: ", lastRecoveryTime < 0 ? "--" : String.format("%.0f", lastRecoveryTime));
        telemetry.addData("last speed up ms: ", lastSpinUpTime < 0 ? "--" : String.format("%.0f", lastSpinUpTime));
        telemetry.addData("PIDF", "P %.3f I %.3f D %.3f F %.3f", SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        telemetry.addLine("----pollen shooter----");
        telemetry.addData("target rpm: ", pollenShooterVelocity * 60 / 28);
        telemetry.addData("actual rpm: ", pollenShooter.getShooterVelocity() * 60 / 28);
        telemetry.addLine("----nectar shooter----");
        telemetry.addData("target rpm: ", nectarShooterVelocity * 60 / 28);
        telemetry.addData("actual rpm: ", nectarShooter.getShooterVelocity() * 60 / 28);

        telemetry.update();
    }

    private void updateShooterPIDF() {
        if (SHOOTER_P != appliedShooterP || SHOOTER_I != appliedShooterI || SHOOTER_D != appliedShooterD || SHOOTER_F != appliedShooterF) {
            pollenShooter.setShooterPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
            nectarShooter.setShooterPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
            appliedShooterP = SHOOTER_P;
            appliedShooterI = SHOOTER_I;
            appliedShooterD = SHOOTER_D;
            appliedShooterF = SHOOTER_F;
        }
    }

    private boolean isPollenAtTargetSpeed() {
        boolean atTarget = pollenShooterVelocity > 0
                && Math.abs(pollenShooter.getShooterVelocity() - pollenShooterVelocity) * 60.0 / 28.0 <= PUSHER_RPM_TOLERANCE;

        if (atTarget) {
            if (!pollenWasAtTarget) {
                pollenAtTargetTimer.reset();
            }
            pollenReady = pollenAtTargetTimer.milliseconds() >= PUSHER_STABLE_TIME_MS;
        } else {
            pollenAtTargetTimer.reset();
            pollenReady = false;
        }
        pollenWasAtTarget = atTarget;
        return pollenReady;
    }

    @Override
    public void stop() {
        pollenShooter.setShooterVelocity(0);
        nectarShooter.setShooterVelocity(0);
        pusher.home();
    }
}
