package graphStateAbstractionTest;

import java.util.ArrayList;
import java.util.List;

import stateAbstractor.PhiSAReal;
import stateAbstractor.StateAbstractor;
import SARealGenerators.qValueGenerator;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
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
		private double valOfInitGroundState; // NOW STORES THE DELTA BETWEEN THE VALUE OF GROUND AND ABSTRACT
		private double valOfInitAbstractState;
		
		
		public EpsilonToNumStatesTuple(double epsilon, int numStates, double valOfInitGroundState, double valOfInitAbstractState) {
			this.eps = epsilon;
			this.numStates = numStates;
			this.valOfInitGroundState = valOfInitGroundState;
			this.valOfInitAbstractState = valOfInitAbstractState;
		}
		@Override
		public String toString() {
			return eps + "\t" + numStates + "\t" + valOfInitGroundState + "\t" + valOfInitAbstractState;
		}
		
		public double getEpsilon() {
			return this.eps;
		}
		
		public int getNumStates() {
			return this.numStates;
		}
		
		public double getValOfGroundInitState() {
			return this.valOfInitGroundState;
		}
		
		public double getValOfInitAbstractState() {
			return this.valOfInitAbstractState;
		}
	}



	public static List<EpsilonToNumStatesTuple> testQPhiStateReduction(GraphDefinedDomain dg, RewardFunction rf, TerminalFunction tf, State initGraphState, double startEpsilon, double endEpsilon, double epsilonIncrement) {
		List<EpsilonToNumStatesTuple> toReturn = new ArrayList<EpsilonToNumStatesTuple>();

		int numOriginalStates = dg.getNumNodes();
		
		for (double epsilon = startEpsilon; epsilon < endEpsilon; epsilon = epsilon + epsilonIncrement) {

			// Ground MDP
			HashableStateFactory hf = new SimpleHashableStateFactory();
			Domain d = dg.generateDomain();
			ValueIteration vi = new ValueIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations);

			GreedyQPolicy groundPolicy = vi.planFromState(initGraphState);
			
			
			PolicyIteration PI = new PolicyIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations, 1);
			PI.setPolicyToEvaluate(groundPolicy);
			PI.planFromState(initGraphState);
			double valueOfInitStateGroundPolicy = PI.value(initGraphState);
			
			// Abstract MDP VI with a PhiQ* for abstraction
			qValueGenerator qGen = new qValueGenerator(d, rf, initGraphState, tf);
			StateAbstractor qPhi = new PhiSAReal(qGen, epsilon, d.getActions());
			GraphDefinedDomain absDG = qPhi.abstractMDP(dg, rf, tf, initGraphState);
			RewardFunction rfA = qPhi.getRewardFunction();
			Domain absD = absDG.generateDomain();
			ValueIteration aVi = new ValueIteration(absD, rfA, tf, VIParams.gamma, new SimpleHashableStateFactory(), VIParams.maxDelta, VIParams.maxIterations);

			State aInitialState =  qPhi.getAbstractInitialState(absD, initGraphState);
			GreedyQPolicy abstractPolicy = aVi.planFromState(aInitialState);	
			
			
			
			//Gather up values for this test iteration
			int numAbstractStates = aVi.getAllStates().size();
			System.out.println("Num abstract states (eps): " + numAbstractStates + " (" + epsilon + ")");

			Policy groundPolicyFromAbstractPolicy = qPhi.getPolicyForGroundMDP(abstractPolicy, absD, d);
			PolicyIteration abstractPI = new PolicyIteration(d, rf, tf, VIParams.gamma, new SimpleHashableStateFactory(), VIParams.maxDelta, VIParams.maxIterations, 1);
			abstractPI.setPolicyToEvaluate(groundPolicyFromAbstractPolicy);
			GreedyQPolicy pol = abstractPI.planFromState(initGraphState);
			
			
			EpisodeAnalysis ea = pol.evaluateBehavior(initGraphState, rf, 20000);
			List<Double> rewards = ea.rewardSequence;
			
			Double val = 0.0;
			if(rewards.contains(1.0)) {
				val = Math.pow(VIParams.gamma, rewards.indexOf(1.0));
				System.out.println("LOC OF 1.0: " + rewards.indexOf(1.0));
				List<State> stateSeq = ea.stateSequence;
				System.out.println("LAST STATE: " + stateSeq.get(stateSeq.size() - 1));
			}
			
			
			double valueOfInitStateAbstractPolicy = abstractPI.value(initGraphState);

			System.out.println("\tVal of abstract init: " + valueOfInitStateAbstractPolicy);
			System.out.println("\tVal of ground init: " + valueOfInitStateGroundPolicy + "\n");
			
			
			EpsilonToNumStatesTuple toAdd = new EpsilonToNumStatesTuple(epsilon, numAbstractStates, valueOfInitStateGroundPolicy, valueOfInitStateAbstractPolicy);
			toReturn.add(toAdd);

		}
		
		System.out.println("Num Original State: " + numOriginalStates);
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
//		double UWStartEpsilon = 0.01;
//		double UWEndEpsilon = 1000;
//		double UWEpsStep = 1;
//		int initStateID = 0;
//		List<EpsilonToNumStatesTuple> upWorldResults = testQPhiStateReduction(upWorldDG, upWorldRF, initStateID, UWStartEpsilon, UWEndEpsilon, UWEpsStep);
//		System.out.println("upWorldResults: ");
//		for (EpsilonToNumStatesTuple x : upWorldResults) {
//			System.out.println(x);
//		}
		//---------END-----------
		
		//---------TRENCH--------
//		int height = 3;
//		int width = 3;
//		
//		TrenchDomainGenerator gen = new TrenchDomainGenerator(height, width);
//		
//		TrenchDomainToGraphDomain graphTrenchMaker = new TrenchDomainToGraphDomain(gen);
//		GraphDefinedDomain trenchGraphDefinedDomain = graphTrenchMaker.createGraphDomain();
//		Domain d = trenchGraphDefinedDomain.generateDomain();
//		
//		RewardFunction graphRF = new GraphRF(graphTrenchMaker.goalStateIDs);
//		TerminalFunction graphTF = new NullTermination();
//		State initGraphState = GraphDefinedDomain.getState(d, graphTrenchMaker.initStateID);
//		double trenchStartEpsilon = 0.64;
//		double trenchEndEpsilon = 25; 
//		List<EpsilonToNumStatesTuple> trenchResults = testQPhiStateReduction(trenchGraphDefinedDomain, graphRF, graphTF, initGraphState, trenchStartEpsilon, trenchEndEpsilon, 2.0);
//		System.out.println("trenchResults: ");
////		DPrint.mode(0);
//		for (EpsilonToNumStatesTuple x : trenchResults) {
//			System.out.println(x);
//		}
		
		//---------END-----------
		

		//--------RANDOMMDP--------
//		int numRandomMDPStates = 1000;
//		int numRandomMDPActions = 3;
//		GraphDefinedDomain randomMDPDG = RandomMDPGenerator.getRandomMDP(numRandomMDPStates, numRandomMDPActions);
//		RewardFunction randomMDPRF = new RandomMDPGenerator.RandomMDPRF(numRandomMDPStates);
//		double RMDPStartEpsilon = 0.001;
//		double RMDPEndEpsilon = 10;
//		double RMDPEpsStep = 1;
//		List<EpsilonToNumStatesTuple> RMDPResults = testQPhiStateReduction(randomMDPDG, randomMDPRF, RMDPStartEpsilon, RMDPEndEpsilon, RMDPEpsStep);
//
//		System.out.println("randomMDPResults: ");
//		for (EpsilonToNumStatesTuple x : RMDPResults) {
//			System.out.println(x);
//		}
	}
}
