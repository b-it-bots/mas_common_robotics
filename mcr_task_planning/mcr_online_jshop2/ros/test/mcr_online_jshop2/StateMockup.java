package mcr_online_jshop2;

import java.util.List;

import mcr_task_planning_msgs.Atom;

import org.ros.internal.message.RawMessage;

public class StateMockup implements mcr_task_planning_msgs.State {
    private List<Atom> atoms;

    @Override
    public RawMessage toRawMessage() {
        return null;
    }

    @Override
    public List<Atom> getAtoms() {
        return atoms;
    }

    @Override
    public void setAtoms(List<Atom> a) {
        atoms = a;
    }

}
