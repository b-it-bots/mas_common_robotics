package mcr_online_jshop2;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.message.MessageListener;

public class Jshop2Node implements NodeMain {

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("jshop2");
    }

    @Override
    public void onStart(ConnectedNode node) {
    }

    @Override
    public void onShutdown(Node node) {
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

}