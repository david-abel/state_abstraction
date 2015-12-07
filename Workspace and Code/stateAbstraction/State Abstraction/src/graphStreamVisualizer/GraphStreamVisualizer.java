package graphStreamVisualizer;

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.List;

import graphStateAbstraction.NStateChainGenerator;
import graphStateAbstraction.Test;
import graphStateAbstraction.UpWorldGenerator;
import graphStateAbstraction.VIParams;

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
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
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
import burlap.oomdp.singleagent.environment.SimulatedEnvironment;
import burlap.oomdp.singleagent.explorer.TerminalExplorer;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

public class GraphStreamVisualizer {
	int numStates;
	RewardFunction rf;
	private Domain d;

	public static String styleSheet = 
			"node {fill-color: grey;size: 40px;}" +
					"edge {shape: cubic-curve;size: 6px;}"		
					;

	public static String[] colorPalette = {
		"rgb(198,255,179)",
		"rgb(179,198,255)",
		"rgb(236,179,255)",
		"rgb(255,179,198)",
		"rgb(198,179,255)",
		"rgb(255,198,179)",
		"rgb(255,236,179)",
		"rgb(236,255,179)",
		"rgb(179,255,236)",
		"rgb(255,179,236)",
		"rgb(179,255,198)",
		"rgb(179,236,255)",
		"rgb(117,152,255)",
		"rgb(56,106,255)",
		"rgb(255,221,117)",
	"rgb(255,205,56)"};

	public GraphStreamVisualizer(Domain d, int numStates, RewardFunction rf) {
		this.numStates = numStates;
		this.rf = rf;
		this.d = d;
	}

	public void render() {	
		//Use advanced viewer
		System.setProperty("org.graphstream.ui.renderer", "org.graphstream.ui.j2dviewer.J2DGraphRenderer");

		Graph graph = new MultiGraph("Tutorial 1");	
		graph.addAttribute("ui.stylesheet", styleSheet);
		Domain dom = this.d;

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

					int otherStateIndex = GraphDefinedDomain.getNodeId(sPrime);

					boolean directed = true;
					DecimalFormat df = new DecimalFormat("#.####");
					df.setRoundingMode(RoundingMode.DOWN);
					String probString = df.format(prob);
					Edge e = graph.addEdge(Integer.toString(stateIndex) + ", " + Integer.toString(otherStateIndex) + ", " + ga.actionName(), Integer.toString(stateIndex), Integer.toString(otherStateIndex), directed);
					int edgeReward = (int) rf.reward(currState, ga, sPrime);
					int actionIndex = dom.getActions().indexOf(a);
					e.addAttribute("ui.label", probString);
					// Set color to match action
					String colorString = "fill-color:" + colorPalette[actionIndex % colorPalette.length]  + ";";
					// Set edge width to mag of reward
					e.addAttribute("ui.style", "size:" + edgeReward + "px;" + colorString);
				}
			}


		}

		Viewer view = graph.display();
	}
	
	public static void addExplorer(Domain d, RewardFunction rf, TerminalFunction tf, State initialState) {
		SimulatedEnvironment env = new SimulatedEnvironment(d, rf, tf, initialState);
		TerminalExplorer exp = new TerminalExplorer(d, env);
		exp.explore();
	}

	public static void main(String[] args) {

		//VI Params
		TerminalFunction tf = new NullTermination();

		// Ground MDP
		int height = 10;
		int width = 5;
		int n =height*width;
		HashableStateFactory hf = new SimpleHashableStateFactory();
		GraphDefinedDomain dg = UpWorldGenerator.getUPWorld(width, height);
		RewardFunction rf = new UpWorldGenerator.UpWorldRF();
		Domain d = dg.generateDomain();
		ValueIteration vi = new ValueIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations);
		State gInitialState = GraphDefinedDomain.getState(d, 0);
		Policy gPol = vi.planFromState(gInitialState);	

		// Abstract MDP VI with a PhiQ* for abstraction
		TerminalFunction tfa = new NullTermination();
		double epsilon = 10;
		qValueGenerator qGen = new qValueGenerator(d, rf, gInitialState);
		StateAbstractor qPhi = new PhiSAReal(qGen, epsilon, d.getActions());
		HashableStateFactory hfA = new SimpleHashableStateFactory();
		GraphDefinedDomain absDG = qPhi.abstractMDP(dg, rf);
		RewardFunction rfA = qPhi.getRewardFunction();
		Domain absD= absDG.generateDomain();
		ValueIteration aVi = new ValueIteration(absD, rfA, tfa, VIParams.gamma, hfA, VIParams.maxDelta, VIParams.maxIterations);
		State aInitialState =  qPhi.getAbstractInitialState(absD, gInitialState);
		GreedyQPolicy abstractPolicy = aVi.planFromState(aInitialState);	

		//Evaluate abstract policy in ground MDP:
		Policy groundPolicyFromAbstractPolicy = qPhi.getPolicyForGroundMDP(abstractPolicy, absD, d);
		PolicyIteration PI = new PolicyIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations, 1);
		PI.setPolicyToEvaluate(groundPolicyFromAbstractPolicy);
		PI.planFromState(gInitialState);
		System.out.println("Value of initial state using abstract MDP: " + PI.value(gInitialState) + " vs value actual value of " + vi.value(gInitialState));

		GraphStreamVisualizer test = new GraphStreamVisualizer(d, n, rf);
		test.render();
		GraphStreamVisualizer test2 = new GraphStreamVisualizer(absD, aVi.getAllStates().size(), rfA);
		test2.render();
//		
//		addExplorer(absD, rfA, tf, aInitialState);
	}

}
