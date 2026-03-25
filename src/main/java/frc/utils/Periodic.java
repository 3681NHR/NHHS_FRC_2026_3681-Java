package frc.utils;

import java.util.LinkedList;

public abstract class Periodic {

    public Periodic(){
        Periodic.registeredClasses.add(this);
    }

    public abstract void update();

    private static LinkedList<Periodic> registeredClasses = new LinkedList<>();

    public static void updateAll(){
        for(Periodic p : registeredClasses){
            p.update();
        }
    }
}
