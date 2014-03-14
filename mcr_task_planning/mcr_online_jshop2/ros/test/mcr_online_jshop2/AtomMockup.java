package mcr_online_jshop2;

import java.util.List;

import org.ros.internal.message.RawMessage;

public class AtomMockup implements mcr_task_planning_msgs.Atom {
    private String name;
    private List<String> parameters;
    
    @Override
    public RawMessage toRawMessage() {
        return null;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public List<String> getParameters() {
        return parameters;
    }

    @Override
    public void setName(String n) {
        name = n;
    }

    @Override
    public void setParameters(List<String> p) {
        parameters = p;
    }
}
