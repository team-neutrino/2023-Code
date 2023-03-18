package frc.robot.util;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class KeyCombination {

    public KeyCombination() {}

    public Command whileTrueButtonCombination(JoystickButton p_button, Command p_command, Command p_comboCommand) {
    if(p_button.getAsBoolean()){
        return p_comboCommand;
    }
    else {
          return p_command;
      }
    }
}
