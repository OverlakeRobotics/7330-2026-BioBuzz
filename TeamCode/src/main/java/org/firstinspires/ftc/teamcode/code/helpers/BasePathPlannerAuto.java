package org.firstinspires.ftc.teamcode.code.helpers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Pusher;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.system.PathPlanAuto;

public abstract class BasePathPlannerAuto extends PathPlanAuto {
    private static final double SHOOTER_RPM = 3000.0;
    private static final double INTAKE_POWER = 0.5;
    private static final double PUSH_DELAY_SECONDS = 1.0;
    private static final int DEFAULT_SHOT_COUNT = 4;

    private Intake intake;
    private Shooter shooter;
    private Pusher pusher;

    private final ElapsedTime pushTimer = new ElapsedTime();
    private int shotsPushed;
    private int shotsToPush = DEFAULT_SHOT_COUNT;

    @Override
    protected final void setup() {
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
        shooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"), DcMotorEx.Direction.REVERSE);
        pusher = new Pusher(hardwareMap.get(Servo.class, "pusher"));
    }

    @Override
    protected final void tags() {
        stopTag("shoot")
                .start(value -> {
                    shotsPushed = 0;
                    shotsToPush = Math.max(1, (int) Math.round(value));
                    intake.setIntakePower(INTAKE_POWER);
                    shooter.setShooterVelocity(SHOOTER_RPM * 28.0 / 60.0);
                    pushTimer.reset();
                })
                .loop(() -> {
                    boolean pusherWasBusy = pusher.getState() != 0;
                    pusher.update(false);

                    if (pusherWasBusy && pusher.getState() == 0) {
                        pushTimer.reset();
                    }

                    if (shotsPushed < shotsToPush && pusher.getState() == 0 && pushTimer.seconds() >= PUSH_DELAY_SECONDS) {
                        pusher.update(true);
                        shotsPushed++;
                    }
                })
                .until(() -> shotsPushed >= shotsToPush && pusher.getState() == 0)
                .end(() -> {
                    intake.setIntakePower(0);
                    shooter.setShooterVelocity(0);
                    pusher.home();
                });

        tag("reset", pusher::home);
    }

    @Override
    protected final void finish() {
        intake.setIntakePower(0);
        shooter.setShooterVelocity(0);
        pusher.home();
    }
}
