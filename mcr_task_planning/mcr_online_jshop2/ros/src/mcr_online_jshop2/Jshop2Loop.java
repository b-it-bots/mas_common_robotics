package mcr_online_jshop2;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import JSHOP2.Plan;
import JSHOP2.Predicate;

class Jshop2Loop extends CancellableLoop {
    private Subscriber<mcr_task_planning_msgs.Task> subscriberTask;
    private Listener<mcr_task_planning_msgs.Task> listenerTask;

    private Subscriber<mcr_task_planning_msgs.State> subscriberState;
    private Listener<mcr_task_planning_msgs.State> listenerState;

    private Subscriber<std_msgs.String> subscriberEventIn;
    private Listener<std_msgs.String> listenerEventIn;
    
    private Publisher<mcr_task_planning_msgs.Plan> publisherPlan;
    private Publisher<std_msgs.String> publisherEventOut;
    private ConnectedNode node;
    private NodeConfiguration nodeConfiguration;
    private MessageFactory messageFactory;
    
    private Jshop2RosConverter converter = new Jshop2RosConverter();
    
    private String jshop2Domain = "";

    public Jshop2Loop(ConnectedNode n) {
        node = n;
    }

    @Override
    protected void setup() {
        listenerTask = new Listener<mcr_task_planning_msgs.Task>();
        subscriberTask = node.newSubscriber("~task", mcr_task_planning_msgs.Task._TYPE);
        subscriberTask.addMessageListener(listenerTask);
        
        listenerState = new Listener<mcr_task_planning_msgs.State>();
        subscriberState = node.newSubscriber("~state", mcr_task_planning_msgs.State._TYPE);
        subscriberState.addMessageListener(listenerState);
        
        listenerEventIn = new Listener<std_msgs.String>();
        subscriberEventIn = node.newSubscriber("~event_in", std_msgs.String._TYPE);
        subscriberEventIn.addMessageListener(listenerEventIn);
        
        publisherPlan = node.newPublisher("~plan", mcr_task_planning_msgs.Plan._TYPE);
        publisherEventOut = node.newPublisher("~event_out", std_msgs.String._TYPE);
        
        nodeConfiguration = NodeConfiguration.newPrivate();
        messageFactory = nodeConfiguration.getTopicMessageFactory();
        
        jshop2Domain = node.getParameterTree().getString("~domain");
    }
    
    private String rosProblemToJshop2Problem(String domainName,
            mcr_task_planning_msgs.State state,
            mcr_task_planning_msgs.Task task) {
        StringBuffer buf = new StringBuffer();
        
        buf.append("(defproblem problem " + domainName + " ");
        buf.append(converter.stateFromRosToJshop2(state));
        buf.append(" (");
        buf.append(converter.taskFromRosToJshop2(task));
        buf.append(") )");
        
        return buf.toString();
    }

    private mcr_task_planning_msgs.Plan convertJshop2PlanToRosPlan(Plan jshop2Plan) {
        mcr_task_planning_msgs.Plan rosPlan = messageFactory.newFromType(mcr_task_planning_msgs.Plan._TYPE);

        String[] primitiveTasks = JSHOP2.JSHOP2.getDomain().getPrimitiveTasks();
        List<mcr_task_planning_msgs.Action> actions = new LinkedList<mcr_task_planning_msgs.Action>();

        for (Predicate p: jshop2Plan.getOps()) {
            mcr_task_planning_msgs.Action action = messageFactory.newFromType(mcr_task_planning_msgs.Action._TYPE);
            String head = primitiveTasks[p.getHead()];
            List<String> parameters = converter.termFromJshop2ToRos(p.getParam());

            action.setName(head);
            action.setParameters(parameters);

            actions.add(action);
        }
        rosPlan.setActions(actions);

        return rosPlan;
    }

    @Override
    protected void loop() throws InterruptedException {
        Thread.sleep(100);

        std_msgs.String rosEvent = listenerEventIn.consumeData();
        if (rosEvent == null) return;

        String event = rosEvent.getData();
        System.out.println("Event: " + event);
        if (!event.equals("e_start")) return;

        std_msgs.String eventOut = messageFactory.newFromType(std_msgs.String._TYPE);
        eventOut.setData("e_failed");

        if ((listenerState.hasData()) && (listenerTask.hasData())) {
            try {
                InputStream streamDomain = new ByteArrayInputStream(jshop2Domain.getBytes("UTF-8"));
                Domain domain = new Domain(streamDomain);
                String domainCode = domain.generate();

                String jshop2Problem = rosProblemToJshop2Problem(
                        domain.getName(), listenerState.consumeData(),
                        listenerTask.consumeData());
                InputStream streamProblem = new ByteArrayInputStream(jshop2Problem.getBytes("UTF-8"));

                // Create the problem description Java code
                Problem problem = new Problem(streamProblem, 1, domain.getDomainDescription());
                String problemCode = problem.generate();

                HashMap<String, String> nameToCode = new HashMap<String, String>();
                nameToCode.put(domain.getName(), domainCode);
                nameToCode.put(problem.getName(), problemCode);

                // Create the online JSHOP planner
                OnlineJSHOP2 shop = new OnlineJSHOP2(nameToCode);
                List<Plan> plans = shop.getPlans();

                if (!plans.isEmpty()) {
                    mcr_task_planning_msgs.Plan plan = convertJshop2PlanToRosPlan(plans.get(0));
                    publisherPlan.publish(plan);
                    eventOut.setData("e_done");
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        publisherEventOut.publish(eventOut);
    }
}