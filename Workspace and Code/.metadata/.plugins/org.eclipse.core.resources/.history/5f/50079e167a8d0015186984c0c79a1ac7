package graphStateAbstraction;

import java.util.List;

import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.oomdp.core.states.State;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.common.UniformCostRF;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

public class Test {

	public static GraphDefinedDomain getNStateChain(int totalNumStates) {
		double slipProb = .8;
		double gamma = .95;

		GraphDefinedDomain dg = new GraphDefinedDomain(totalNumStates);

		// Set up transitions
		for (int i = 0; i < totalNumStates-1; i++) {
			((GraphDefinedDomain) dg).setTransition(i, 0/*action*/, i+1, slipProb);
			((GraphDefinedDomain) dg).setTransition(i, 0/*action*/, 0, 1.0-slipProb);
			((GraphDefinedDomain) dg).setTransition(i, 1/*action*/, 0, slipProb);
			((GraphDefinedDomain) dg).setTransition(i, 1/*action*/, i+1, 1.0-slipProb);
		}

		// Set up last state transition
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 0/*action*/, totalNumStates-1, slipProb);
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 1/*action*/, totalNumStates-1, 1.0-slipProb);
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 1/*action*/, 0, slipProb);
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 0/*action*/, 0, 1.0-slipProb);

		return dg;
	}	

	public static void main(String[] args) {
		//VI Params
		int maxIterations = 10000;
		double maxDelta = .0001;
		double gamma = .95;
		TerminalFunction tf = new NullTermination();
		
		// Ground MDP -- 5 state chain with uniform negative reward b/c why not...
		HashableStateFactory hf = new SimpleHashableStateFactory();
		GraphDefinedDomain dg = getNStateChain(5);
		RewardFunction rf = new UniformCostRF();
		Domain d = dg.generateDomain();
		ValueIteration vi = new ValueIteration(d, rf, tf, gamma, hf, maxDelta, maxIterations);
		State gInitialState = GraphDefinedDomain.getState(d, 0);
		vi.planFromState(gInitialState);	
		System.out.println("Ground initial state value: " + vi.value(gInitialState));
		
		// Abstract MDP VI with a PhiModel for abstraction
		HashableStateFactory hfA = new SimpleHashableStateFactory();
		StateAbstractor phiMod = new PhiModel();
		GraphDefinedDomain absDG = phiMod.abstractMDP(dg, rf);
		RewardFunction rfA = phiMod.getRewardFunction();
		Domain absD= absDG.generateDomain();
		ValueIteration aVi = new ValueIteration(absD, rfA, tf, gamma, hfA, maxDelta, maxIterations);
		State aInitialState = GraphDefinedDomain.getState(absD, 0);
		aVi.planFromState(aInitialState);	
		System.out.println("Abstract initial state value: " + aVi.value(aInitialState));

	}



}
