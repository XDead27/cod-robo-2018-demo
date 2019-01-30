package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Crater", group = "Autonomie")
public final class AutonomousCrater extends AutonomousTest {

    @Override
    protected void runOp() {
        ChooseAndPushCube();

        sleep(1000);

        rotit(-60);

        walk_with_obstacle_and_range(15, false);

        rotit(90);

        setWheelsPower(0.7, 0.7);
        //aici va faceti de cap baieti cu functiile din AutonomousTest
    }

    @Override
    protected void endOp() {
        //extindem mecanismele cumva
    }
}
