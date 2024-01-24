package eu.qrobotics.centerstage.teamcode.AutoUtil;

import java.util.ArrayList;

public class LogicNode {
    public ArrayList<Edge> edgesList;

    private String ID;

    public LogicNode(String givenID) {
        this.ID = givenID;
        this.edgesList = new ArrayList<>();
    }

    public interface Condition {
        boolean condition();
    }

    private class Edge {
        Condition TransitionCondition;
        Runnable PreTransitionAction;
        LogicNode TransitionNode;

        public Edge(Condition condition, Runnable transitionAction, LogicNode targetNode) {
            this.TransitionCondition = condition;
            this.PreTransitionAction = transitionAction;
            this.TransitionNode = targetNode;
        }
    }

    public void addCondition(Condition condition, Runnable transitionAction, LogicNode targetNode) {
        edgesList.add(new Edge(condition, transitionAction, targetNode));
    }

    public void run() {
        for (Edge edge : edgesList) {
            if (edge.TransitionCondition.condition()) {
                edge.PreTransitionAction.run();
                transition(edge.TransitionNode);
            }
        }
    }

    private void transition(LogicNode node) {
        edgesList = node.edgesList;
        ID = node.ID;
    }

    public String getID() {
        return ID;
    }
}
