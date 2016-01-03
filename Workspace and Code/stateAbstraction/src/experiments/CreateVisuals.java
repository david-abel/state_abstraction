package experiments;

import graphStateAbstractionTest.VIParams;
import graphStreamVisualizer.GraphStreamVisualizer;
import stateAbstractor.PhiSAReal;
import stateAbstractor.StateAbstractor;
import SARealGenerators.qValueGenerator;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;
import domains.GraphRF;
import domains.NormalDomainToGraphDomain;
import domains.nchain.NChainGenerator;
import domains.taxi.TaxiDomainGenerator;
import domains.trench.TrenchDomainGenerator;
import domains.upworld.UpWorldGenerator;

/**
 * Imeplementation to visualize an arbitrary MDP domain as a graph
 * and for compressing under any of our approximate abstraction schemes
 * (and visualizing).
 * @author David Abel.
 *
 */
public class CreateVisuals {

	/**
	 * Function that compresses a GraphDefinedDomain under phi_{Q^*}, for the given epsilon.
	 * @param graphDefinedDomain
	 * @param graphTF
	 * @param graphRF
	 * @param initGraphState
	 * @param epsilon
	 */
	public static void compressAndVisualizeMDP(GraphDefinedDomain graphDefinedDomain, TerminalFunction graphTF, RewardFunction graphRF, State initGraphState, double epsilon) {
		// Retrieve the graph domain.
		Domain graphDomain = graphDefinedDomain.generateDomain();
		
		// Run vi to compute Q*.
		ValueIteration vi = new ValueIteration(graphDomain, graphRF, graphTF, VIParams.gamma, new SimpleHashableStateFactory(), VIParams.maxDelta, VIParams.maxIterations);
		Policy gPi = vi.planFromState(initGraphState);
		
		// Abstract the MDP.
		qValueGenerator qGen = new qValueGenerator(graphDomain, graphRF, initGraphState, graphTF);
		StateAbstractor qPhi = new PhiSAReal(qGen, epsilon, graphDomain.getActions());
		GraphDefinedDomain abstractDG = qPhi.abstractMDP(graphDefinedDomain, graphRF, graphTF, initGraphState);
		RewardFunction abstractRF = qPhi.getRewardFunction();
		Domain abstractDomain = abstractDG.generateDomain();
		ValueIteration aVi = new ValueIteration(abstractDomain, abstractRF, graphTF, VIParams.gamma, new SimpleHashableStateFactory(), VIParams.maxDelta, VIParams.maxIterations);
		State aInitialState =  qPhi.getAbstractInitialState(abstractDomain, initGraphState);
		GreedyQPolicy abstractPolicy = aVi.planFromState(aInitialState);
		
		// Evaluate.
		Policy groundPolicyFromAbstractPolicy = qPhi.getPolicyForGroundMDP(abstractPolicy, abstractDomain, graphDomain);
		PolicyIteration PI = new PolicyIteration(graphDomain, graphRF, graphTF, VIParams.gamma, new SimpleHashableStateFactory(), VIParams.maxDelta, VIParams.maxIterations, 1);
		PI.setPolicyToEvaluate(groundPolicyFromAbstractPolicy);
		PI.planFromState(initGraphState);
		System.out.println("Value of initial state using abstract MDP: " + PI.value(initGraphState) + " vs value actual value of " + vi.value(initGraphState));

		// Visualize.
		GraphStreamVisualizer groundMDPVisualizer = new GraphStreamVisualizer(graphDomain, vi.getAllStates().size(), graphRF);
		groundMDPVisualizer.render(null, false);
		GraphStreamVisualizer abstractMDPVisualizer = new GraphStreamVisualizer(abstractDomain, aVi.getAllStates().size(), abstractRF);
		abstractMDPVisualizer.render(qPhi, true);
	}
	
	/**
	 * Function that compresses an arbitrary DomainGenerator by first converting it into a GraphDefinedDomain
	 * And then compresses and visualizes.
	 * @param gen
	 * @param tf
	 * @param rf
	 * @param initialState
	 * @param epsilon
	 */
	public static void convertCompressAndVisualizeMDP(DomainGenerator gen, TerminalFunction tf, RewardFunction rf, State initialState, double epsilon) {
		
		// Convert to graph domain.
		NormalDomainToGraphDomain graphMaker = new NormalDomainToGraphDomain(gen, tf, rf, initialState);
		GraphDefinedDomain graphDefinedDomain = graphMaker.createGraphDomain();
		TerminalFunction graphTF = new NullTermination();
		Domain graphDomain = graphDefinedDomain.generateDomain();
		RewardFunction graphRF = new GraphRF(graphDomain, rf, tf, graphMaker.graphIndexToNonGraphState, graphMaker.graphActionIndexToNonGraphAction);
		State initGraphState = GraphDefinedDomain.getState(graphDomain, graphMaker.initStateID);
		
		// Compress and visualize.
		compressAndVisualizeMDP(graphDefinedDomain, graphTF, graphRF, initGraphState, epsilon);
	}
	
	public static void main(String[] args) {
		
		// Change the text here to switch between visuals.
		String domainToVisualize = "TRENCH"; // One of "TRENCH", "UPWORLD", "NCHAIN", or add your own.
		double epsilon = .05;
		
		if (domainToVisualize == "TRENCH") {
			// Create trench domain.
			int height = 3;
			int width = 3;
			DomainGenerator gen = new TrenchDomainGenerator(height, width);
			TerminalFunction tf = new TrenchDomainGenerator.TrenchTF(height - 1, width - 1);
			RewardFunction rf = new TrenchDomainGenerator.TrenchRF(height - 1, width - 1);
			Domain oldDomain = gen.generateDomain();
			State initialState = ((TrenchDomainGenerator) gen).getInitialState(oldDomain);
			convertCompressAndVisualizeMDP(gen, tf, rf, initialState, epsilon);
		}
		else if (domainToVisualize == "UPWORLD") {
			// Create upworld domain.
			int height = 10;
			int width = 4;
			GraphDefinedDomain gen = UpWorldGenerator.getUPWorld(width, height);
			Domain d = gen.generateDomain();
			State initialState = GraphDefinedDomain.getState(d, 0);
			TerminalFunction tf = new NullTermination();
			RewardFunction rf = new UpWorldGenerator.UpWorldRF();
			compressAndVisualizeMDP((GraphDefinedDomain) gen, tf, rf, initialState, epsilon);
		}
		else if (domainToVisualize == "TAXI") {
			// Create taxi domain.
			TaxiDomainGenerator taxiGen = new TaxiDomainGenerator();
			Domain d = taxiGen.generateDomain();
			TerminalFunction tf = new TaxiDomainGenerator.TaxiTF();
			RewardFunction rf = new TaxiDomainGenerator.TaxiRF();
			State initialTaxiState = taxiGen.getInitialState(d);
			convertCompressAndVisualizeMDP(taxiGen, tf, rf, initialTaxiState, epsilon);
		}
		else {
			// Create nchain domain.
			int numStates = 20;
			GraphDefinedDomain gen = NChainGenerator.getNStateChain(numStates);
			Domain d = gen.generateDomain();
			State initialState = GraphDefinedDomain.getState(d, 0);
			TerminalFunction tf = new NullTermination();
			RewardFunction rf = new NChainGenerator.nStateChainRF(numStates);
			compressAndVisualizeMDP(gen, tf, rf, initialState, epsilon);
		}
		
		
	}
	
}
