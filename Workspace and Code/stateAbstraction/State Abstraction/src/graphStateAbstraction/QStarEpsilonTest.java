package graphStateAbstraction;

import java.util.ArrayList;
import java.util.List;

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

public class QStarEpsilonTest {
	public static class EpsilonToNumStatesTuple {
		private double eps;
		private int numStates;
		private double valOfInitialState;
		public EpsilonToNumStatesTuple(double epsilon, int numStates, double valOfInitialState) {
			this.eps = epsilon;
			this.numStates = numStates;
			this.valOfInitialState = valOfInitialState;
		}
		@Override
		public String toString() {
			return eps + "\t" + numStates + "\t" + valOfInitialState;
		}
	}


	public static List<EpsilonToNumStatesTuple> testQPhiStateReduction(GraphDefinedDomain dg, RewardFunction rf, double startEpsilon, double endEpsilon, double epsilonStep) {
		List<EpsilonToNumStatesTuple> toReturn = new ArrayList<EpsilonToNumStatesTuple>();

		for (double epsilon = startEpsilon; epsilon < endEpsilon; epsilon = epsilon*2) {

			//VI Params
			TerminalFunction tf = new NullTermination();

			// Ground MDP
			int n = 100;
			HashableStateFactory hf = new SimpleHashableStateFactory();
			Domain d = dg.generateDomain();
			ValueIteration vi = new ValueIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations);
			State gInitialState = GraphDefinedDomain.getState(d, 0);
			Policy gPol = vi.planFromState(gInitialState);	

			// Abstract MDP VI with a PhiQ* for abstraction
			qValueGenerator qGen = new qValueGenerator(d, rf, gInitialState);
			StateAbstractor qPhi = new PhiSAReal(qGen, epsilon, d.getActions());
			HashableStateFactory hfA = new SimpleHashableStateFactory();
			GraphDefinedDomain absDG = qPhi.abstractMDP(dg, rf);
			RewardFunction rfA = qPhi.getRewardFunction();
			Domain absD= absDG.generateDomain();
			ValueIteration aVi = new ValueIteration(absD, rfA, tf, VIParams.gamma, hfA, VIParams.maxDelta, VIParams.maxIterations);
			State aInitialState =  qPhi.getAbstractInitialState(absD, gInitialState);
			GreedyQPolicy abstractPolicy = aVi.planFromState(aInitialState);	

			//Gather up values for this test iteration
			int numAbstractStates = aVi.getAllStates().size();

			Policy groundPolicyFromAbstractPolicy = qPhi.getPolicyForGroundMDP(abstractPolicy, absD, d);
			PolicyIteration PI = new PolicyIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations, 1);
			PI.setPolicyToEvaluate(groundPolicyFromAbstractPolicy);
			PI.planFromState(gInitialState);
			double valueOfInitialState = PI.value(gInitialState);

			EpsilonToNumStatesTuple toAdd = new EpsilonToNumStatesTuple(epsilon, numAbstractStates, valueOfInitialState);
			toReturn.add(toAdd);

		}	
		return toReturn;
	}

	public static void main(String[] args) {
		//--------NSTATE CHAIN--------
		//		int nChainNumStates = 1000;
		//		GraphDefinedDomain nStateChainDG = NStateChainGenerator.getNStateChain(nChainNumStates);
		//		RewardFunction nChainRF = new NStateChainGenerator.nStateChainRF(nChainNumStates);
		//		double nChainStartEpsilon = 0.001;
		//		double nChainEndEpsilon = 10;
		//		double nChainEpsStep = 1;
		//
		//		List<EpsilonToNumStatesTuple> nStateChainResults = testQPhiStateReduction(nStateChainDG, nChainRF, nChainStartEpsilon, nChainEndEpsilon, nChainEpsStep);
		//		System.out.println("nStateChainResults: ");
		//		for (EpsilonToNumStatesTuple x : nStateChainResults) {
		//			System.out.println(x);
		//		}

		//--------UPWORLD--------
//		int upWorldWidth = 50;
//		int upWorldHeight = 20;
//		GraphDefinedDomain upWorldDG = UpWorldGenerator.getUPWorld(upWorldWidth, upWorldHeight);
//		RewardFunction upWorldRF = new UpWorldGenerator.UpWorldRF();
//		double UWStartEpsilon = 0.001;
//		double UWEndEpsilon = 10;
//		double UWEpsStep = 1;
//		List<EpsilonToNumStatesTuple> upWorldResults = testQPhiStateReduction(upWorldDG, upWorldRF, UWStartEpsilon, UWEndEpsilon, UWEpsStep);
//
//		System.out.println("upWorldResults: ");
//		for (EpsilonToNumStatesTuple x : upWorldResults) {
//			System.out.println(x);
//		}

		//--------RANDOMMDP--------
		int numRandomMDPStates = 1000;
		int numRandomMDPActions = 3;
		GraphDefinedDomain randomMDPDG = RandomMDPGenerator.getRandomMDP(numRandomMDPStates, numRandomMDPActions);
		RewardFunction randomMDPRF = new RandomMDPGenerator.RandomMDPRF(numRandomMDPStates);
		double RMDPStartEpsilon = 0.001;
		double RMDPEndEpsilon = 10;
		double RMDPEpsStep = 1;
		List<EpsilonToNumStatesTuple> RMDPResults = testQPhiStateReduction(randomMDPDG, randomMDPRF, RMDPStartEpsilon, RMDPEndEpsilon, RMDPEpsStep);

		System.out.println("randomMDPResults: ");
		for (EpsilonToNumStatesTuple x : RMDPResults) {
			System.out.println(x);
		}
	}
}
