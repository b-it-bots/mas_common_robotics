package mcr_online_jshop2;

import static org.junit.Assert.*;

import java.util.LinkedList;
import java.util.List;

import mcr_online_jshop2.Jshop2RosConverter;
import mcr_task_planning_msgs.Atom;
import mcr_task_planning_msgs.State;
import mcr_task_planning_msgs.Task;

import org.junit.Test;

public class Jshop2RosConverterTest {

    @Test
    public void testConstruction() {
        @SuppressWarnings("unused")
        Jshop2RosConverter converter = new Jshop2RosConverter();
    }

    @Test
    public void testRosToJshop2WithEmptyTask() {
        Jshop2RosConverter converter = new Jshop2RosConverter();
        Task t = new TaskMockup();

        assertEquals("()", converter.taskFromRosToJshop2(t));
    }

    @Test
    public void testRosToJshop2WithTaskAndNoParameters() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        Task t = new TaskMockup();

        t.setName("task1");
        assertEquals("(task1)", convert.taskFromRosToJshop2(t));

        t.setName("task2");
        assertEquals("(task2)", convert.taskFromRosToJshop2(t));
    }

    @Test
    public void testRosToJshop2WithTaskAndParameters() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        Task t = new TaskMockup();

        t.setName("task1");

        LinkedList<String> parameters = new LinkedList<String>();
        parameters.add("param1");
        t.setParameters(parameters);
        assertEquals("(task1 param1)", convert.taskFromRosToJshop2(t));

        parameters.add("param2");
        t.setParameters(parameters);
        assertEquals("(task1 param1 param2)", convert.taskFromRosToJshop2(t));
    }

    @Test
    public void testRosToJshop2WithoutTaskAndWithParameters() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        Task t = new TaskMockup();

        LinkedList<String> parameters = new LinkedList<String>();
        parameters.add("param1");
        t.setParameters(parameters);
        assertEquals("()", convert.taskFromRosToJshop2(t));

        t.setName("");
        assertEquals("()", convert.taskFromRosToJshop2(t));

        t.setName(" abc def ");
        assertEquals("()", convert.taskFromRosToJshop2(t));
    }



    @Test
    public void testRosToJshop2WithEmptyAtom() {
        Jshop2RosConverter converter = new Jshop2RosConverter();
        Atom a = new AtomMockup();

        assertEquals("()", converter.atomFromRosToJshop2(a));
    }

    @Test
    public void testRosToJshop2WithAtomAndNoParameters() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        Atom a = new AtomMockup();

        a.setName("task1");
        assertEquals("(task1)", convert.atomFromRosToJshop2(a));

        a.setName("task2");
        assertEquals("(task2)", convert.atomFromRosToJshop2(a));
    }

    @Test
    public void testRosToJshop2WithAtomAndParameters() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        Atom a = new AtomMockup();

        a.setName("task1");

        LinkedList<String> parameters = new LinkedList<String>();
        parameters.add("param1");
        a.setParameters(parameters);
        assertEquals("(task1 param1)", convert.atomFromRosToJshop2(a));

        parameters.add("param2");
        a.setParameters(parameters);
        assertEquals("(task1 param1 param2)", convert.atomFromRosToJshop2(a));
    }

    @Test
    public void testRosToJshop2WithoutAtomAndWithParameters() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        Atom a = new AtomMockup();

        LinkedList<String> parameters = new LinkedList<String>();
        parameters.add("param1");
        a.setParameters(parameters);
        assertEquals("()", convert.atomFromRosToJshop2(a));

        a.setName("");
        assertEquals("()", convert.atomFromRosToJshop2(a));

        a.setName(" abc def ");
        assertEquals("()", convert.atomFromRosToJshop2(a));
    }



    @Test
    public void testRosToJshop2EmptyState() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        State s = new StateMockup();

        assertEquals("()", convert.stateFromRosToJshop2(s));
    }

    @Test
    public void testRosToJshop2ValidStates() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        State s = new StateMockup();
        Atom a = new AtomMockup();
        a.setName("atom1");

        List<Atom> atoms = new LinkedList<Atom>();
        atoms.add(a);
        s.setAtoms(atoms);

        assertEquals("((atom1))", convert.stateFromRosToJshop2(s));
    }
}
