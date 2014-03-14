package mcr_online_jshop2;

import java.util.LinkedList;
import java.util.List;

import JSHOP2.Term;
import JSHOP2.TermConstant;
import JSHOP2.TermList;
import mcr_task_planning_msgs.Atom;
import mcr_task_planning_msgs.State;
import mcr_task_planning_msgs.Task;

public class Jshop2RosConverter {

    /**
     * Convert a ROS mcr_task_planning_msgs task message to a string
     * representation which can be provided to JSHOP2.
     * 
     * @param t The ROS representation of the task.
     * 
     * @return A JSHOP2 compatible string representation of the task.
     */
    public String taskFromRosToJshop2(Task t) {
        StringBuffer buf = new StringBuffer();
        String name = t.getName();

        buf.append("(");
        if ((name != null) && (!name.equals("")) && (!name.contains(" "))) {
            buf.append(name);
            if (t.getParameters() != null) {
                for (String p : t.getParameters()) {
                    buf.append(" " + p);
                }
            }
        }
        buf.append(")");

        return buf.toString();
    }

    /**
     * Convert a ROS mcr_task_planning_msgs atom message to a string
     * representation which can be provided to JSHOP2.
     * 
     * @param t The ROS representation of the atom.
     * 
     * @return A JSHOP2 compatible string representation of the atom.
     */
    public String atomFromRosToJshop2(Atom a) {
        StringBuffer buf = new StringBuffer();
        String name = a.getName();

        buf.append("(");
        if ((name != null) && (!name.equals("")) && (!name.contains(" "))) {
            buf.append(name);
            if (a.getParameters() != null) {
                for (String p : a.getParameters()) {
                    buf.append(" " + p);
                }
            }
        }
        buf.append(")");

        return buf.toString();
    }

    /**
     * Convert a ROS mcr_task_planning_msgs state message to a string
     * representation which can be provided to JSHOP2.
     * 
     * @param t The ROS representation of the state.
     * 
     * @return A JSHOP2 compatible string representation of the state.
     */
    public String stateFromRosToJshop2(State s) {
        StringBuffer buf = new StringBuffer();

        buf.append("(");
        if (s.getAtoms() != null) {
            for (Atom a: s.getAtoms()) {
                buf.append(atomFromRosToJshop2(a));
            }
        }
        buf.append(")");

        return buf.toString();
    }

    /**
     * Convert a JSHOP2 term to a list of strings.
     * 
     * @param term The JSHOP2 term.
     * 
     * @return The list of strings representing the term.
     */
    public List<String> termFromJshop2ToRos(Term term) {
        List<String> parameters = new LinkedList<String>();

        if (term == null) return parameters;

        if (term instanceof TermConstant) {
            TermConstant tc = (TermConstant)term;

            parameters.add(tc.toString());
        } else if (term instanceof TermList) {
            TermList tl = (TermList)term;

            parameters.add(tl.getList().getHead().toString());

            if (tl.getList().getTail() != TermList.NIL) {
                parameters.addAll(termFromJshop2ToRos(tl.getList().getTail()));
            }
        }

        return parameters;
    }
}
