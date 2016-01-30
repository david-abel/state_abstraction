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
				List<State> stateSeq = ea.stateSequence;
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

	}
}