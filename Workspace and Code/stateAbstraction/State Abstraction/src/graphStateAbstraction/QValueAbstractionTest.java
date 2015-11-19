package graphStateAbstraction;

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
public class QValueAbstractionTest {

	public static void main(String[] args) {
		//VI Params
		TerminalFunction tf = new NullTermination();

		// Ground MDP
		int n = 100;
		HashableStateFactory hf = new SimpleHashableStateFactory();
		GraphDefinedDomain dg = NStateChainGenerator.getNStateChain(n);
		RewardFunction rf = new NStateChainGenerator.nStateChainRF(n);
		Domain d = dg.generateDomain();
		ValueIteration vi = new ValueIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations);
		State gInitialState = GraphDefinedDomain.getState(d, 0);
		Policy gPol = vi.planFromState(gInitialState);	
		System.out.println("Ground initial state value: " + vi.value(gInitialState));

		// Abstract MDP VI with a PhiQ* for abstraction
		double epsilon = 1000;
		qValueGenerator qGen = new qValueGenerator(d, rf, gInitialState);
		StateAbstractor qPhi = new PhiSAReal(qGen, epsilon, d.getActions());
		HashableStateFactory hfA = new SimpleHashableStateFactory();
		GraphDefinedDomain absDG = qPhi.abstractMDP(dg, rf);
		RewardFunction rfA = qPhi.getRewardFunction();
		Domain absD= absDG.generateDomain();
		ValueIteration aVi = new ValueIteration(absD, rfA, tf, VIParams.gamma, hfA, VIParams.maxDelta, VIParams.maxIterations);
		State aInitialState = GraphDefinedDomain.getState(absD, 0);
		GreedyQPolicy abstractPolicy = aVi.planFromState(aInitialState);	
		

		//Evaluate abstract policy in ground MDP:
		Policy groundPolicyFromAbstractPolicy = qPhi.getPolicyForGroundMDP(abstractPolicy, absDG, dg);
		PolicyIteration PI = new PolicyIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations, 1);
		PI.setPolicyToEvaluate(groundPolicyFromAbstractPolicy);
		PI.planFromState(gInitialState);
		System.out.println("Value of initial state using abstract MDP: " + PI.value(gInitialState) + " vs value actual value of " + vi.value(gInitialState));

		// Print actions for all states
//		for (int i = 0; i < n; i++) {
//			State s = GraphDefinedDomain.getState(d, i);
//			System.out.println("Action for state " + i + " is " +  groundPolicyFromAbstractPolicy.getAction(s).actionName() + " vs " + gPol.getAction(s).actionName() + " from the ground MDP");
//		}
	}
}