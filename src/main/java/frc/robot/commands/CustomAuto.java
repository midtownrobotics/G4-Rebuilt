package frc.robot.commands;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map; 

public class CustomAuto {
    private final AutoFactory m_autoFactory;
    private final Autos m_autos;
    private final AutoChooser m_autoChooser;
    private final RobotCommands m_robotCommands;

    private final SendableChooser<position> m_start_chooser = new SendableChooser<>();
    private final SendableChooser<side> m_starting_side_chooser = new SendableChooser<>();
    private final SendableChooser<CustomAutoRoute> m_first_path_chooser = new SendableChooser<>();
    private final SendableChooser<CustomAutoRoute> m_second_path_chooser = new SendableChooser<>();

    private Map<CustomAutoRoute, Segment> segmentMap = new LinkedHashMap<>();

    public CustomAuto(Drive drivetrain, RobotCommands robotCommands){
        m_autoFactory = new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::followPath,
                true,
                drivetrain);
        m_autos = new Autos(m_autoFactory, drivetrain, robotCommands);
        m_autoChooser = new AutoChooser("Do Nothing");
        m_robotCommands = robotCommands;

        segmentMap.put(new CustomAutoRoute(position.HubStart, position.Shoot, route.Depot), this::hubDepotShoot);
        segmentMap.put(new CustomAutoRoute(position.Shoot, position.Shoot, route.Depot), this::shootDepotShoot);
        segmentMap.put(new CustomAutoRoute(position.Shoot, position.Shoot, route.Shoot), this::shoot);
        segmentMap.put(new CustomAutoRoute(position.TrenchStart, position.Shoot, route.Neutral), this::trenchNeutralShoot);
        segmentMap.put(new CustomAutoRoute(position.Shoot, position.Shoot, route.Neutral), this::shootNeutralShoot);

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
        m_second_path_chooser.setDefaultOption("None", null);
        for(CustomAutoRoute autoRoute: segmentMap.keySet()){
            String name = autoRoute.startPos().toString();
            name = name.concat(" ").concat(autoRoute.endPos().toString());
            name = name.concat(" ").concat(autoRoute.route().toString());
            if(autoRoute.startPos() == position.Shoot && autoRoute.endPos() == position.Shoot && autoRoute.route() == route.Shoot){
                name = "Shoot";
            }
            m_first_path_chooser.addOption(name, autoRoute);
            m_second_path_chooser.addOption(name, autoRoute);
        }
        SmartDashboard.putData("Custom Auto First Path Chooser", m_first_path_chooser);
        SmartDashboard.putData("Custom Auto Second Path Chooser", m_second_path_chooser);
    }

    private void setAutoOptions(int index){

    }

    AutoRoutine custom(){
        AutoRoutine routine = m_autoFactory.newRoutine("Custom");

        List<Command> legs = new ArrayList<>();
        for(SendableChooser<CustomAutoRoute> chooser: List.of(m_first_path_chooser, m_second_path_chooser)){
            Segment segment = segmentMap.get(chooser.getSelected());
            if(segment == null){
                continue;
            }
            legs.add(segment.build(routine, legs.isEmpty()));
        }

        routine.active().onTrue(Commands.sequence(legs.toArray(Command[]::new)));

        return routine;
    }

    private interface Segment {
        Command build(AutoRoutine routine, boolean resetOdometry);
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

    private Command hubDepotShoot(AutoRoutine routine, boolean resetOdometry){
        AutoTrajectory HubDepotShoot = routine.trajectory("CustomAuto_HubDepotShoot");

        return Commands.sequence(
            resetOdometry ? HubDepotShoot.resetOdometry() : Commands.none(),
            HubDepotShoot.cmd());
    }

    private Command shootDepotShoot(AutoRoutine routine, boolean resetOdometry) {
        AutoTrajectory ShootDepotShoot = routine.trajectory("CustomAuto_ShootDepotShoot");

        return Commands.sequence(
                resetOdometry ? ShootDepotShoot.resetOdometry() : Commands.none(),
                ShootDepotShoot.cmd());
    }

    private Command shoot(AutoRoutine routine, boolean resetOdometry){
        return Commands.sequence(
            m_robotCommands.dump().withTimeout(5.0));
    }

    private Command trenchNeutralShoot(AutoRoutine routine, boolean resetOdometry) {
        AutoTrajectory TrenchNeutralShoot = routine.trajectory("CustomAuto_TrenchNeutralShoot");

        if (m_starting_side_chooser.getSelected() == side.Right) {
            TrenchNeutralShoot = TrenchNeutralShoot.mirrorY();
        }

        return Commands.sequence(
                resetOdometry ? TrenchNeutralShoot.resetOdometry() : Commands.none(),
                TrenchNeutralShoot.cmd());
    }

    private Command shootNeutralShoot(AutoRoutine routine, boolean resetOdometry) {
        AutoTrajectory ShootNeutralShoot = routine.trajectory("CustomAuto_ShootNeutralShoot");

        if (m_starting_side_chooser.getSelected() == side.Right) {
            ShootNeutralShoot = ShootNeutralShoot.mirrorY();
        }

        return Commands.sequence(
                resetOdometry ? ShootNeutralShoot.resetOdometry() : Commands.none(),
                ShootNeutralShoot.cmd());
    }


}

record CustomAutoRoute(CustomAuto.position startPos, CustomAuto.position endPos, CustomAuto.route route){}
