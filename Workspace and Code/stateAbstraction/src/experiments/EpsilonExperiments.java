package experiments;

import graphStateAbstractionTest.QStarEpsilonTest;
import graphStateAbstractionTest.QStarEpsilonTest.EpsilonToNumStatesTuple;

import java.util.ArrayList;
import java.util.List;

import domains.GraphRF;
import domains.GraphTF;
import domains.NormalDomainToGraphDomain;
import domains.nchain.NChainGenerator;
import domains.taxi.TaxiDomainGenerator;
import domains.trench.TrenchDomainGenerator;
import domains.trench.TrenchDomainToGraphDomain;
import domains.upworld.UpWorldGenerator;
import stateAbstractor.PhiModel;
import stateAbstractor.PhiSAReal;
import stateAbstractor.StateAbstractor;
import plotting.Plotter;
import SARealGenerators.qValueGenerator;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.oomdp.core.states.State;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.common.UniformCostRF;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

/**
 * Code for running experiments for our ICML paper.
 * @author David Abel
 *
 */
public class EpsilonExperiments {
	
	/**
	 * Given a DomainGenerator + RF + TF + initState, this method generates abstract MDPs subject to different epsilons, and prints the results.
	 * @param domainName
	 * @param gen
	 * @param tf
	 * @param rf
	 * @param initialState
	 */
	public static void generateEpsilonResults(GraphDefinedDomain graphDefinedDomain, TerminalFunction graphTF, RewardFunction graphRF, State initGraphState) {
		
		// Iterate over epsilon and compute the number of states.
		double startEpsilon = 0.0;
		double endEpsilon = 5.0;
		double epsilonIncrement = 0.5;
		List<EpsilonToNumStatesTuple> epsilonAndNumStatesPairs = QStarEpsilonTest.testQPhiStateReduction(graphDefinedDomain, graphRF, graphTF, initGraphState, startEpsilon, endEpsilon, epsilonIncrement);
		
		List<Double> epsilons = new ArrayList<Double>();
		List<Integer> numStates = new ArrayList<Integer>();
		
		System.out.println("results: ");
		for (EpsilonToNumStatesTuple x : epsilonAndNumStatesPairs) {
			epsilons.add(x.getEpsilon());
			numStates.add(x.getNumStates());
			
			System.out.println(x);
		}
		
		// Now make the plot...
	}
	
	public static void compressAndGenEpsilonResults(DomainGenerator gen, TerminalFunction tf, RewardFunction rf, State initialState) {
		// Convert to graph domain.
		NormalDomainToGraphDomain graphMaker = new NormalDomainToGraphDomain(gen, tf, rf, initialState);
		GraphDefinedDomain graphDefinedDomain = graphMaker.createGraphDomain();
		Domain d = graphDefinedDomain.generateDomain();
		RewardFunction graphRF = new GraphRF(graphMaker.goalStateIDs);
		TerminalFunction graphTF = new GraphTF(graphMaker.goalStateIDs);
		State initGraphState = GraphDefinedDomain.getState(d, graphMaker.initStateID);
		
		generateEpsilonResults(graphDefinedDomain, graphTF, graphRF, initGraphState);
	}
	
	public static void main(String[] args) {

		// Create trench domain.
		int height = 4;
		int width = 3;
		TrenchDomainGenerator trenchGen = new TrenchDomainGenerator(height, width);
		TerminalFunction trenchTF = new TrenchDomainGenerator.TrenchTF(height - 1, width - 1);
		RewardFunction trenchRF = new TrenchDomainGenerator.TrenchRF(height - 1, width - 1);
		Domain oldTrenchDomain = trenchGen.generateDomain();
		State initialTrenchState = trenchGen.getInitialState(oldTrenchDomain);
//		generateEpsilonResults(trenchGen, trenchTF, trenchRF, initialTrenchState);
		
		// Create taxi domain.
		TaxiDomainGenerator taxiGen = new TaxiDomainGenerator();
		TerminalFunction taxiTF = new TaxiDomainGenerator.TaxiTF();
		RewardFunction taxiRF = new TaxiDomainGenerator.TaxiRF();
		Domain oldTaxiDomain = taxiGen.generateDomain();
		State initialTaxiState = taxiGen.getInitialState(oldTaxiDomain);
//		generateEpsilonResults(taxiGen, taxiTF, taxiRF, initialTaxiState);
		
		
		// Create upworld domain.
		int upWorldHeight = 100;
		int upWorldWidth = 10;
		GraphDefinedDomain upWorldGen = UpWorldGenerator.getUPWorld(upWorldWidth, upWorldHeight);
		Domain upWorldDomain = upWorldGen.generateDomain();
		State initialUpWorldState = GraphDefinedDomain.getState(upWorldDomain, 0);
		TerminalFunction upWorldTF = new NullTermination();
		RewardFunction upWorldRF = new UpWorldGenerator.UpWorldRF();
//		generateEpsilonResults(upWorldGen, upWorldTF, upWorldRF, initialUpWorldState);
		
		
		// Create nchain domain.
		int numStates = 20;
		GraphDefinedDomain nChainGen = NChainGenerator.getNStateChain(numStates);
		Domain nChainDomain = nChainGen.generateDomain();
		State initialNChainState = GraphDefinedDomain.getState(nChainDomain, 0);
		TerminalFunction nChainTF = new NullTermination();
		RewardFunction nChainRF = new NChainGenerator.nStateChainRF(numStates);
		generateEpsilonResults(nChainGen, nChainTF, nChainRF, initialNChainState);

	}
}