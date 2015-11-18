package graphStreamVisualizer;

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.List;

import graphStateAbstraction.Test;

import org.graphstream.graph.Edge;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.*;
import org.graphstream.ui.view.Viewer;

import stateAbstractor.PhiSAReal;
import stateAbstractor.StateAbstractor;
import SARealGenerators.qValueGenerator;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

public class GraphStreamVisualizer {
	private GraphDefinedDomain d;
	int numStates;
	RewardFunction rf;

	public GraphStreamVisualizer(GraphDefinedDomain d, int numStates, RewardFunction rf) {
		this.d = d;
		this.numStates = numStates;
		this.rf = rf;
	}

	public void render() {	
		Graph graph = new MultiGraph("Tutorial 1");	
		Domain dom = d.generateDomain();

		for (int stateIndex = 0; stateIndex < numStates; stateIndex++) {
			//Add nodes for each state.
			Node node = graph.addNode(Integer.toString(stateIndex));
			node.addAttribute("ui.label", Integer.toString(stateIndex));
		}


		//Add transition probabilities/edges.
		for (int stateIndex = 0; stateIndex < numStates; stateIndex++) {
			//Add nodes for each state.
			State currState = GraphDefinedDomain.getState(dom, stateIndex);

			for (Action a : dom.getActions()) {
				GroundedAction ga = a.getAssociatedGroundedAction();

				List<TransitionProbability> tProbs = ga.getTransitions(currState);
				for (TransitionProbability tProb : tProbs) {
					State sPrime = tProb.s;
					double prob = tProb.p;
					//TODO ADD PROBABILITY

					int otherStateIndex = GraphDefinedDomain.getNodeId(sPrime);

					//TODO DONT SKIP SELF LOOPS
//					if (stateIndex != otherStateIndex) {
						boolean directed = true;
						DecimalFormat df = new DecimalFormat("#.#");
						df.setRoundingMode(RoundingMode.DOWN);
						String probString = df.format(prob);
						Edge e = graph.addEdge(Integer.toString(stateIndex) + Integer.toString(otherStateIndex) + ga.actionName(), Integer.toString(stateIndex), Integer.toString(otherStateIndex), directed);
						e.addAttribute("ui.label", probString + ", " + rf.reward(currState, ga, sPrime));
//					}
				}
			}


		}

		Viewer view = graph.display();
	}

	public static void main(String[] args) {
		//VI Params
		int maxIterations = 10000;
		double maxDelta = .0001;
		double gamma = .9;
		TerminalFunction tf = new NullTermination();

		// Ground MDP
		int n = 100;
		HashableStateFactory hf = new SimpleHashableStateFactory();
		GraphDefinedDomain dg = Test.getNStateChain(n);
		RewardFunction rf = new Test.nStateChainRF(n);
		Domain d = dg.generateDomain();
		ValueIteration vi = new ValueIteration(d, rf, tf, gamma, hf, maxDelta, maxIterations);
		State gInitialState = GraphDefinedDomain.getState(d, 0);
		Policy gPol = vi.planFromState(gInitialState);	
		System.out.println("Ground initial state value: " + vi.value(gInitialState));

		// Abstract MDP VI with a PhiQ* for abstraction
		double epsilon = 1;
		qValueGenerator qGen = new qValueGenerator(d, rf, gamma, gInitialState);
		StateAbstractor qPhi = new PhiSAReal(qGen, epsilon, d.getActions());
		HashableStateFactory hfA = new SimpleHashableStateFactory();
		GraphDefinedDomain absDG = qPhi.abstractMDP(dg, rf);
		RewardFunction rfA = qPhi.getRewardFunction();
		Domain absD= absDG.generateDomain();
		ValueIteration aVi = new ValueIteration(absD, rfA, tf, gamma, hfA, maxDelta, maxIterations);
		State aInitialState = GraphDefinedDomain.getState(absD, 0);
		GreedyQPolicy abstractPolicy = aVi.planFromState(aInitialState);	

		
		
		GraphStreamVisualizer test = new GraphStreamVisualizer(dg, n, rf);
//		GraphStreamVisualizer test = new GraphStreamVisualizer(absDG, aVi.getAllStates().size(), rfA);
		test.render();
	}

}
