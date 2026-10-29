package org.firstinspires.ftc.teamcode.code.helpers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.code.parts.Pusher;

@TeleOp(name = "bioBuzz26", group = "TeleOp")
public class BaseTeleOp extends OpMode{

    private BasicHolonomicDrivetrain driveTrain;
    private Intake intake;
    private Shooter pollenShooter;
    private Servo pollenServo;
    private Pusher pollenPusher;
    public static double driveVelocity = 2800;
    private final double intakePower = 0.5;
    //private static double pollenPower = idk;

    public void init() {
        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
        pollenShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"));
        pollenPusher = new Pusher(pollenServo,-65.0,5.0,200.0);
    }

    public void loop() {
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * driveVelocity);
        driveTrain.drive();

        if(gamepad1.y) {
            intake.setIntakePower(intakePower);
        }
        if(gamepad1.a) {
            intake.setIntakePower(-1 * intakePower);
        }
        if(gamepad1.x) {
            pollenShooter.setShooterVelocity(pollenPower);
        }
        if(gamepad1.b) {
            pollenShooter.setShooterVelocity(-1 * pollenPower);
        }

        pollenPusher.update(gamepad1.right_trigger_pressed);

    }
}
