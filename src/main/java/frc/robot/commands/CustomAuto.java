package frc.robot.commands;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Map; 

public class CustomAuto {
    private final AutoFactory m_autoFactory;
    private final Autos m_autos;
    private final AutoChooser m_autoChooser;

    private final SendableChooser<position> m_start_chooser = new SendableChooser<>();
    private final SendableChooser<side> m_starting_side_chooser = new SendableChooser<>();
    private final SendableChooser<CustomAutoRoute> m_first_path_chooser = new SendableChooser<>();
    private final SendableChooser<CustomAutoRoute> m_second_path_chooser = new SendableChooser<>();

    private Map<CustomAutoRoute, AutoRoutine> routineMap = new HashMap<>();

    public CustomAuto(Drive drivetrain, RobotCommands robotCommands){
        m_autoFactory = new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::followPath,
                true,
                drivetrain);
        m_autos = new Autos(m_autoFactory, drivetrain, robotCommands);
        m_autoChooser = new AutoChooser("Do Nothing");

        routineMap.put(new CustomAutoRoute(position.HubStart, position.Shoot, route.Depot), HubDepotShoot());

        generateAutoChooser();
        smartDashboardInit();
    }

    private void generateAutoChooser() {
        m_autoChooser.addRoutine("Madtown Left", m_autos::MadtownLeft);
        m_autoChooser.addRoutine("Madtown Right", m_autos::MadtownRight);
        m_autoChooser.addRoutine("Hub Swipe Left", m_autos::HubSwipeLeft);
        m_autoChooser.addRoutine("Hub Swipe Right", m_autos::HubSwipeRight);
        m_autoChooser.addRoutine("1002 Left", m_autos::copy1002left);
        m_autoChooser.addRoutine("1002 Right", m_autos::copy1002right);
        m_autoChooser.addRoutine("match 13 depot", m_autos::centerDepot);
        m_autoChooser.addRoutine("right hub clean up", m_autos::rightHubCleanUp);
        m_autoChooser.addRoutine("Custom", this::custom);
        RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler());
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    public void smartDashboardInit(){
        m_start_chooser.setDefaultOption("Trench", position.TrenchStart);
        m_start_chooser.addOption("Bump", position.BumpStart);
        m_start_chooser.addOption("Hub", position.HubStart);
        SmartDashboard.putData("Custom Auto Start Chooser", m_start_chooser);

        m_starting_side_chooser.setDefaultOption("Left", side.Left);
        m_starting_side_chooser.addOption("Right", side.Right);
        SmartDashboard.putData("Starting side", m_starting_side_chooser);

        m_first_path_chooser.setDefaultOption("None", null);
        for(CustomAutoRoute autoRoute: routineMap.keySet()){
            m_first_path_chooser.addOption("test", autoRoute);
        }
        SmartDashboard.putData("Custom Auto First Path Chooser", m_first_path_chooser);
        SmartDashboard.putData("Custom Auto Second Path Chooser", m_second_path_chooser);
    }

    AutoRoutine custom(){
        AutoRoutine routine = m_autoFactory.newRoutine("Custom");

        AutoRoutine first_routine = routineMap.get(m_first_path_chooser.getSelected());

        routine.active().onTrue(
            Commands.sequence(
                first_routine.cmd()));

        return routine;
    }

    enum position{
        TrenchStart, BumpStart, HubStart, Shoot
    }

    enum side{
        Left, Right
    }

    enum route{
        Neutral, Depot, Shoot
    }

    AutoRoutine HubDepotShoot(){
        AutoRoutine routine = m_autoFactory.newRoutine("HubDepotShoot");

        AutoTrajectory HubDepotShoot = routine.trajectory("CustomAuto_HubDepotShoot");

        routine.active().onTrue(
        Commands.sequence(
            HubDepotShoot.resetOdometry(),
            HubDepotShoot.cmd()));

        return routine;
    }
}

record CustomAutoRoute(CustomAuto.position startPos, CustomAuto.position endPos, CustomAuto.route route){}
