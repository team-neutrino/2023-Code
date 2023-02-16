package frc.robot.util;

import frc.robot.Constants;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

public class PoseProcessor {

  public static String absoluteAppend(String p_filename) {
    String ret =
        System.getProperty("user.dir") + Constants.UtilConstants.UNIVERSAL_DIRECTORY + p_filename;
    return ret;
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
        for (int i = 0; i < 3; ++i) {
          coordList[i] = Double.valueOf(parsedLine[i]);
        }
        PoseTriplet poseTriplet = new PoseTriplet(coordList[0], coordList[1], coordList[2]);
        poseArray.add(poseTriplet);
      }
      scanner.close();
    } catch (FileNotFoundException e) {
      System.out.println("Scanner reading error");
      e.printStackTrace();
    }
    return poseArray;
  }
}
