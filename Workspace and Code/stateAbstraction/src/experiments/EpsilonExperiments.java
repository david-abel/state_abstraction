package experiments;

import graphStateAbstractionTest.QStarEpsilonTest;
import graphStateAbstractionTest.QStarEpsilonTest.EpsilonToNumStatesTuple;

import java.util.List;

import domains.GraphRF;
import domains.GraphTF;
import domains.NormalDomainToGraphDomain;
import domains.nchain.NStateChainGenerator;
import domains.trench.TrenchDomainGenerator;
import domains.trench.TrenchDomainToGraphDomain;
import stateAbstractor.PhiModel;
import stateAbstractor.PhiSAReal;
import stateAbstractor.StateAbstractor;
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
 * An example of Q* abstraction in an 5 state chain MDP.
 * @author Dhershkowitz
 *
 */
public class EpsilonExperiments {

	public static void createEpsilonVsPerformancePlot() {
		
	}
	
	public static void createEpsilonVsNumStatesPlot(DomainGenerator gen, TerminalFunction tf, RewardFunction rf, State initialState) {
		
		// Convert to graph domain.
		NormalDomainToGraphDomain graphMaker = new NormalDomainToGraphDomain(gen, tf, rf, initialState);
		GraphDefinedDomain graphDefinedDomain = graphMaker.createGraphDomain();
		RewardFunction graphRF = new GraphRF(graphMaker.goalStateIDs);
		TerminalFunction graphTF = new GraphTF(graphMaker.goalStateIDs);
		
		// Iterate over epsilon and compute the number of states.
		double startEpsilon = 0.64;
		double endEpsilon = 500; 
		List<EpsilonToNumStatesTuple> epsilonAndNumStatesPairs = QStarEpsilonTest.testQPhiStateReduction(graphDefinedDomain, graphRF, graphTF, graphMaker.initStateID, startEpsilon, endEpsilon);
		System.out.println("results: ");
		for (EpsilonToNumStatesTuple x : epsilonAndNumStatesPairs) {
			System.out.println(x);
		}
		
		
	}
	
	public static void main(String[] args) {

		// Create trench domain.
		int height = 3;
		int width = 3;
		TrenchDomainGenerator gen = new TrenchDomainGenerator(height, width);
		TerminalFunction tf = new TrenchDomainGenerator.TrenchTF(height - 1, width - 1);
		RewardFunction rf = new TrenchDomainGenerator.TrenchRF(height - 1, width - 1);
		Domain oldDomain = gen.generateDomain();
		State initialState = gen.getInitialState(oldDomain);
		
		// Compress and compare epsilon to number of states.
		createEpsilonVsNumStatesPlot(gen, tf, rf, initialState);

		
	}
}