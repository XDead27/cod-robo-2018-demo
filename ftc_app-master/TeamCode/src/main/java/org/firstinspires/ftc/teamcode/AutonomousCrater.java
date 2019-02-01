package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Crater", group = "Autonomie")
public final class AutonomousCrater extends AutonomousTest {

    @Override
    protected void runOp() {
        ChooseAndPushCube(true);

        //sleep(1000);

        optimizare();
        //aici va faceti de cap baieti cu functiile din AutonomousTest
    }

    @Override
    protected void endOp() {
        //extindem mecanismele cumva
        ridicare_perii_encoder(-10, 0.2);
    }
}
