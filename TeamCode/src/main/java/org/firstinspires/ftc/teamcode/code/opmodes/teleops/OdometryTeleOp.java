package org.firstinspires.ftc.teamcode.code.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name = "Odemetry TeleOp", group = "TeleOp")
public class OdometryTeleOp extends BasicTeleOp {
    private static final Pose2D START_POSITION =
            new Pose2D(DistanceUnit.INCH, -63, 63, AngleUnit.DEGREES, 0);

    private final Pose2D[] presets = {
            new Pose2D(DistanceUnit.INCH, -63, 15, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 63, 15, AngleUnit.DEGREES, 180),
            new Pose2D(DistanceUnit.INCH, 21, 63, AngleUnit.DEGREES, -90)
    };

    private int poseIndex = 0;
    private boolean drivingToPose = false;

    @Override
    protected Pose2D getDefaultStartPosition() {
        return START_POSITION;
    }

    @Override
    protected void updateDrive() {
        driveTrain.updatePosition();

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
    }

    @Override
    protected boolean isNectarTogglePressed() {
        // X starts a preset drive here, so D-pad up toggles the nectar shooter.
        return gamepad1.dpadUpWasPressed();
    }

    @Override
    protected String getNectarToggleControlName() {
        return "d-pad up";
    }

    @Override
    protected void updateAdditionalShooterControls() {
        if (gamepad1.rightBumperWasPressed()) {
            pollenShooterVelocity -= 100 * TICKS_PER_SEC;
        } else if (gamepad1.leftBumperWasPressed()) {
            pollenShooterVelocity += 100 * TICKS_PER_SEC;
        }
    }

    @Override
    protected void addDriveTelemetry() {
        Pose2D currentPos = driveTrain.getPosition();
        Pose2D selectedPose = presets[poseIndex];

        telemetry.addLine("D-pad left/right: select pose | X: drive to pose | sticks: manual drive/cancel");
        telemetry.addData("selected pose", "%d / %d", poseIndex + 1, presets.length);
        telemetry.addData("target pose", "X: %.1f, Y: %.1f, H: %.1f",
                selectedPose.getX(DistanceUnit.INCH),
                selectedPose.getY(DistanceUnit.INCH),
                selectedPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("driving to pose", drivingToPose);
        telemetry.addData("current pose", "X: %.1f, Y: %.1f, H: %.1f",
                currentPos.getX(DistanceUnit.INCH),
                currentPos.getY(DistanceUnit.INCH),
                currentPos.getHeading(AngleUnit.DEGREES));
    }
}
