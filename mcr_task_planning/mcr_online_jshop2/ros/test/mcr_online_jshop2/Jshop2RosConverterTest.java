package mcr_online_jshop2;

import static org.junit.Assert.*;

import java.util.LinkedList;

import mcr_online_jshop2.Jshop2RosConverter;
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

}
