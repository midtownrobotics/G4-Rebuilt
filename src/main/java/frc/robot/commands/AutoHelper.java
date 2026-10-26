package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public class AutoHelper {
    static Pose2d poseFromChoreoVar(String name){
        try {
            File deployDir = Filesystem.getDeployDirectory();
            File chorFile = new File(deployDir, "choreo/Choreo.chor");

            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(chorFile);

            JsonNode poseNode = root.get("variables").get("poses").get(name);

            Pose2d pose = new Pose2d(
            poseNode.at("/x/val").asDouble(), 
            poseNode.at("/y/val").asDouble(), 
            Rotation2d.fromRadians(poseNode.at("/heading/val").asDouble()));
            
            return pose;
        } catch (IOException e) {
            e.printStackTrace();
            return new Pose2d();
        }
    }
}
