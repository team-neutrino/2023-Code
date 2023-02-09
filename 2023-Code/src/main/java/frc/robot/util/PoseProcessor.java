package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

import frc.robot.Constants;

public class PoseProcessor {

  private static String absoluteAppend(String p_filename) {
    return System.getProperty("user.dir") + Constants.UtilConstants.UNIVERSAL_DIRECTORY + p_filename;
  }

  public static ArrayList<PoseTriplet> poseTripletsFromFile(String p_filename) {
    ArrayList<PoseTriplet> poseArray = new ArrayList<PoseTriplet>();
    try {
      File poseFile = new File(absoluteAppend(p_filename));
      Scanner scanner = new Scanner(poseFile);
      while (scanner.hasNextLine()) {
        String fullLine = scanner.nextLine();
        String[] parsedLine = fullLine.split(",");
        Double[] coordList = new Double[3];
        for (int i = 0; i < 3; ++i) coordList[i] = Double.valueOf(parsedLine[i]);
        PoseTriplet poseTriplet = new PoseTriplet(coordList[0], coordList[1], coordList[2]);
        poseArray.add(poseTriplet);
      }
    } catch (FileNotFoundException e) {
      System.out.println("Error from PoseProcessor.java");
      e.printStackTrace();
    }
    return poseArray;
  }
}
