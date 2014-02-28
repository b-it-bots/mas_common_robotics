package mcr_online_jshop2;

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

}
