package frc.robot.autos;

import frc.utils.Periodic;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.LinkedList;
import java.util.ListIterator;

public class AutoGenerator extends Periodic {

    LinkedList<Path> sequence = new LinkedList<>();
    LoggedDashboardChooser<Path> chooser = new LoggedDashboardChooser<>("auto/selector");
    LoggedNetworkBoolean enter = new LoggedNetworkBoolean("SmartDashboard/auto/enter", false);
    LoggedNetworkBoolean back = new LoggedNetworkBoolean("SmartDashboard/auto/back", false);
    Path[] options;

    public AutoGenerator(){
        populateChooser(Path.START, true);
        generateText();
        generateRoute();
    }

    @Override
    public void update(){
        if(enter.getAsBoolean() && chooser.get() != null){
            enter(chooser.get());
            enter.set(false);
        }
        if(back.getAsBoolean()){

            if(!sequence.isEmpty())
                sequence.removeLast();
            if(!sequence.isEmpty()){
                populateChooser(sequence.getLast(), false);
            } else {
                populateChooser(Path.START, true);
            }
            generateText();
            generateRoute();

            back.set(false);
        }
    }
    private void enter(Path e){
        if(chooser.get() != null) {
            sequence.add(chooser.get());
            populateChooser(chooser.get(), false);
            generateText();
            generateRoute();
        }
    }

    private void populateChooser(Path point, boolean startingPoint){
        chooser.getSendableChooser().close();
        chooser = new LoggedDashboardChooser<>("auto/selector");
//        chooser.onChange(this::enter);
        for(Path p : point.options){
            if(startingPoint){
                chooser.addOption(String.format("start at %s", p.end), p);
            }else {
                chooser.addOption(String.format("%s via: %s", p.end, p.name), p);
            }

        }
    }

    private void generateText() {
        if(!sequence.isEmpty()){

            StringBuilder out = new StringBuilder(sequence.getFirst().end);
            if (sequence.size() > 1) {
                ListIterator<Path> i = sequence.listIterator(1);
                while (i.hasNext()) {
                    Path p = i.next();
                    out.append(String.format(" =>(%s) %s", p.name, p.end));
                }
            }
            Logger.recordOutput("auto/generated", out.toString());
        } else {
            Logger.recordOutput("auto/generated", "select a starting point first!");
        }
        System.out.println(sequence.size());
    }

    private void generateRoute(){

    }
}
