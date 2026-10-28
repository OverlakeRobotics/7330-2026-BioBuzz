package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


// This OpMode shows a simple example of driving the bot along a set path of points using the
// driveTrain.setPositionDrive() method while passing in an array of Pose2Ds.
@Config
@TeleOp(name = "Odometry Test", group = "Test")
public class OdometryTest extends OpMode {
    // Change to your actual offsets. See GoBildaPinpointDriver.setOffsets() for details on measuring offsets.
    public double yOffset = -168.0;
    public double xOffset = -84.0;

    public static int velocity = 2000;
    public static double tolerance = 4;

    private OdometryHolonomicDrivetrain driveTrain;
    private GoBildaPinpointDriver pinpointDriver;

    private final Pose2D[] drivePoints1 = {
            new Pose2D(DistanceUnit.INCH, 0, 24, AngleUnit.DEGREES, 0),
    };

    private final Pose2D[] drivePoints2 = {
            new Pose2D(DistanceUnit.INCH, 0, 72, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 180)
    };

    private final Pose2D[] drivePoints3 = {
            new Pose2D(DistanceUnit.INCH, 0, 72, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, -90),
            new Pose2D(DistanceUnit.INCH, 72, 0, AngleUnit.DEGREES, 180),
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90),
    };

    private Pose2D[] path = drivePoints1;
    private boolean running = false;
    private boolean lastA = false;

    @Override
    public void init() {
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);

        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        driveTrain.setTolerance(tolerance);
        driveTrain.setVelocity(velocity);
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        driveTrain.drive();

        if (!running) {
            if (gamepad1.x) path = drivePoints1;
            if (gamepad1.y) path = drivePoints2;
            if (gamepad1.b) path = drivePoints3;

            if (gamepad1.a && !lastA) {
                driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                driveTrain.setPositionDrive(path);
                running = true;
            }
        }

        if (running) {
            Pose2D target = path[path.length - 1];
            Pose2D current = pinpointDriver.getPosition();
            double distance = Math.hypot(target.getX(DistanceUnit.INCH) - current.getX(DistanceUnit.INCH), target.getY(DistanceUnit.INCH) - current.getY(DistanceUnit.INCH));

            if (distance <= tolerance) running = false;
        }

        lastA = gamepad1.a;

        telemetry.addData("path", path == drivePoints1 ? 1 : path == drivePoints2 ? 2 : 3);
        telemetry.addData("running:", running);
        telemetry.update();
    }

    @Override
    public void start() {
        // wait for user to start
    }
}