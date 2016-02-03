package experiments;

import java.util.ArrayList;
import java.util.List;

import graphStateAbstractionTest.VIParams;
import graphStreamVisualizer.GraphStreamVisualizer;
import stateAbstractor.PhiAStar;
import stateAbstractor.PhiSAReal;
import stateAbstractor.StateAbstractor;
import SARealGenerators.qValueGenerator;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.options.Option;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;
import domains.GraphRF;
import domains.NormalDomainToGraphDomain;
import domains.minefield.MinefieldGenerator;
import domains.nchain.NChainGenerator;
import domains.randommdp.RandomMDPGenerator;
import domains.taxi.GetPassengerOptionMaker;
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


	// Change the text here to switch between visuals.
	public static String domainToVisualize = "MINEFIELD"; // One of "TRENCH", "UPWORLD", "NCHAIN", "TAXI", "MINEFIELD" or add your own.
	public static double epsilon = 1.1;

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
		//		StateAbstractor qPhi = new PhiSAReal(qGen, epsilon, graphDomain.getActions());
		StateAbstractor qPhi = new PhiAStar(qGen, epsilon, graphDomain.getActions());
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

	public static void convertCompressAndVisualizeMDPOptions(DomainGenerator gen, TerminalFunction tf, RewardFunction rf, State initialState, double epsilon, List<Option> options) {

		// Convert to graph domain.
		NormalDomainToGraphDomain graphMaker = new NormalDomainToGraphDomain(gen, tf, rf, initialState);
		GraphDefinedDomain graphDefinedDomain = graphMaker.createGraphDomain(options);
		TerminalFunction graphTF = new NullTermination();
		Domain graphDomain = graphDefinedDomain.generateDomain();
		RewardFunction graphRF = new GraphRF(graphDomain, rf, tf, graphMaker.graphIndexToNonGraphState, graphMaker.graphActionIndexToNonGraphAction);
		State initGraphState = GraphDefinedDomain.getState(graphDomain, graphMaker.initStateID);

		// Compress and visualize.
		compressAndVisualizeMDP(graphDefinedDomain, graphTF, graphRF, initGraphState, epsilon);
	}

	public static void main(String[] args) {


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
			List<Option> taxiOptions = taxiGen.getOptions(d);
			convertCompressAndVisualizeMDPOptions(taxiGen, tf, rf, initialTaxiState, epsilon, taxiOptions);
		}
		else if (domainToVisualize == "MINEFIELD") {
			// Create upworld domain.
			int minefieldHeight = 10;
			int minefieldWidth = 4;
			int numMineStates = 5;
			GraphDefinedDomain minefieldGen = MinefieldGenerator.getMinefield(minefieldHeight, minefieldWidth);
			Domain minefieldDomain = minefieldGen.generateDomain();
			State initialMinefieldState = GraphDefinedDomain.getState(minefieldDomain, 0);
			RewardFunction minefieldRF = new MinefieldGenerator.MinefieldRF(numMineStates, minefieldHeight * minefieldWidth, minefieldHeight, minefieldWidth);
			compressAndVisualizeMDP((GraphDefinedDomain) minefieldGen, new NullTermination(), minefieldRF, initialMinefieldState, epsilon);
		}
		else if (domainToVisualize == "RANDOM") {
			int numRandStates = 100;
			int numRandActions = 3;
			GraphDefinedDomain randGen = RandomMDPGenerator.getRandomMDP(numRandStates, numRandActions);
			Domain randDomain = randGen.generateDomain();
			State initialRandState = RandomMDPGenerator.getInitialState(randDomain);
			RewardFunction randRF = new RandomMDPGenerator.RandomMDPRF(numRandStates);
			compressAndVisualizeMDP((GraphDefinedDomain) randGen, new NullTermination(), randRF, initialRandState, epsilon);
		}
		else if (domainToVisualize == "NCHAIN") {
			// Create nchain domain.
			int numStates = 10;
			GraphDefinedDomain gen = NChainGenerator.getNStateChain(numStates);
			Domain d = gen.generateDomain();
			State initialState = GraphDefinedDomain.getState(d, 0);
			TerminalFunction tf = new NullTermination();
			RewardFunction rf = new NChainGenerator.nStateChainRF(numStates);
			compressAndVisualizeMDP(gen, tf, rf, initialState, epsilon);
		}


	}

}
