package mcr_online_jshop2;

import static org.junit.Assert.*;

import java.util.LinkedList;
import java.util.List;

import mcr_online_jshop2.Jshop2RosConverter;
import mcr_task_planning_msgs.Atom;
import mcr_task_planning_msgs.State;
import mcr_task_planning_msgs.Task;

import org.junit.Test;

import JSHOP2.Term;
import JSHOP2.TermConstant;
import JSHOP2.TermList;

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



    @Test
    public void testJshop2ToRosNullTerm() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        List<String> data = convert.termFromJshop2ToRos(null);

        assertEquals(0, data.size());
    }

    @Test
    public void testJshop2ToRosSimpleTermConstant() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        JSHOP2.Domain domain = new DomainMockup();
        String[] constants = new String[] { "const" };
        Term t = new TermConstant(0);

        domain.setProblemConstants(constants);
        JSHOP2.JSHOP2.initialize(domain, null);

        List<String> data = convert.termFromJshop2ToRos(t);

        assertEquals(1, data.size());
        assertEquals("const", data.get(0));
    }

    @Test
    public void testJshop2ToRosTermListWithOneElement() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        JSHOP2.Domain domain = new DomainMockup();
        String[] constants = new String[] { "const" };
        Term t = new TermConstant(0);
        TermList tl = new TermList(t, TermList.NIL);

        domain.setProblemConstants(constants);
        JSHOP2.JSHOP2.initialize(domain, null);

        List<String> data = convert.termFromJshop2ToRos(tl);

        assertEquals(1, data.size());
        assertEquals("const", data.get(0));
    }

    @Test
    public void testJshop2ToRosTermListWithTwoElements() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        JSHOP2.Domain domain = new DomainMockup();
        String[] constants = new String[] { "const0", "const1" };
        Term t0 = new TermConstant(0);
        Term t1 = new TermConstant(1);
        TermList tl = new TermList(t0, t1);

        domain.setProblemConstants(constants);
        JSHOP2.JSHOP2.initialize(domain, null);

        List<String> data = convert.termFromJshop2ToRos(tl);

        assertEquals(2, data.size());
        assertEquals("const0", data.get(0));
        assertEquals("const1", data.get(1));
    }

    @Test
    public void testJshop2ToRosTermListWithRecursiveList() {
        Jshop2RosConverter convert = new Jshop2RosConverter();
        JSHOP2.Domain domain = new DomainMockup();
        String[] constants = new String[] { "const0", "const1", "const2" };
        Term t0 = new TermConstant(0);
        Term t1 = new TermConstant(1);
        Term t2 = new TermConstant(2);
        TermList tl = new TermList(t0, new TermList(t1, t2));

        domain.setProblemConstants(constants);
        JSHOP2.JSHOP2.initialize(domain, null);

        List<String> data = convert.termFromJshop2ToRos(tl);

        assertEquals(3, data.size());
        assertEquals("const0", data.get(0));
        assertEquals("const1", data.get(1));
        assertEquals("const2", data.get(2));
    }
}
