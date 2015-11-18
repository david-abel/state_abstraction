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
		int maxIterations = 10000;
		double maxDelta = .0001;
		double gamma = .9;
		TerminalFunction tf = new NullTermination();

		// Ground MDP
		int n = 5;
		HashableStateFactory hf = new SimpleHashableStateFactory();
		GraphDefinedDomain dg = Test.getNStateChain(n);
		RewardFunction rf = new Test.nStateChainRF(n);
		Domain d = dg.generateDomain();
		ValueIteration vi = new ValueIteration(d, rf, tf, gamma, hf, maxDelta, maxIterations);
		State gInitialState = GraphDefinedDomain.getState(d, 0);
		Policy gPol = vi.planFromState(gInitialState);	
		System.out.println("Ground initial state value: " + vi.value(gInitialState));

		// Abstract MDP VI with a PhiQ* for abstraction
		double epsilon = 10;
		qValueGenerator qGen = new qValueGenerator(d, rf, gamma, gInitialState);
		StateAbstractor qPhi = new PhiSAReal(qGen, epsilon, d.getActions());
		HashableStateFactory hfA = new SimpleHashableStateFactory();
		GraphDefinedDomain absDG = qPhi.abstractMDP(dg, rf);
		RewardFunction rfA = qPhi.getRewardFunction();
		Domain absD= absDG.generateDomain();
		ValueIteration aVi = new ValueIteration(absD, rfA, tf, gamma, hfA, maxDelta, maxIterations);
		State aInitialState = GraphDefinedDomain.getState(absD, 0);
		GreedyQPolicy abstractPolicy = aVi.planFromState(aInitialState);	
		
		Policy groundPolicyFromAbstractPolicy = qPhi.getPolicyForGroundMDP(abstractPolicy, absDG, dg);

		//Evaluate abstract policy in ground MDP:
		double maxVIDelta = .0001;
		int maxVIIterations = 1000;
		PolicyIteration PI = new PolicyIteration(d, rf, tf, gamma, hf, maxVIDelta, maxVIIterations, 1);
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