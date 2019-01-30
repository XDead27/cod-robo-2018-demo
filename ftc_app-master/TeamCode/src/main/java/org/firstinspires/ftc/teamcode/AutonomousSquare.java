package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomie Patrat", group = "Autonomie")
public final class AutonomousSquare extends AutonomousTest {

    @Override
    protected void runOp() {
        //aici va faceti de cap baieti cu functiile din AutonomousTest
        switch(ChooseAndPushCube()){
            case 1:
                mers_encoder(25);

                PlopTotem();

                rotit(90);

                walk_with_obstacle_and_range(10, false);
                break;

            case 2:
                walk_with_obstacle_and_range(10, false);

                rotit(-90);

                PlopTotem();

                rotit(180);
                break;

            case 3:
                walk_with_obstacle_and_range(10, false);

                rotit(90);

                PlopTotem();

                rotit(180);

                break;
        }

        sleep(1000);

        walk_with_obstacle_and_range(20, false);

        stopWheels();
    }

    @Override
    protected void endOp() {
        //extindem mecanismele cumva
    }
}
