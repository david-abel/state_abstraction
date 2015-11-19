package graphStateAbstraction;

import stateAbstractor.PhiModel;
import stateAbstractor.StateAbstractor;
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

public class Test {


	/**
	 * An example of phiModel (currently not actually implemented).
	 * @param args
	 */
	public static void main(String[] args) {
		//VI Params

		TerminalFunction tf = new NullTermination();
		
		// Ground MDP
		int n = 5;
		HashableStateFactory hf = new SimpleHashableStateFactory();
		GraphDefinedDomain dg = NStateChainGenerator.getNStateChain(n);
		RewardFunction rf = new NStateChainGenerator.nStateChainRF(n);
		Domain d = dg.generateDomain();
		ValueIteration vi = new ValueIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations);
		State gInitialState = GraphDefinedDomain.getState(d, 0);
		vi.planFromState(gInitialState);	
		System.out.println("Ground initial state value: " + vi.value(gInitialState));
		
		// Abstract MDP VI with a PhiModel for abstraction
		HashableStateFactory hfA = new SimpleHashableStateFactory();
		StateAbstractor phiMod = new PhiModel();
		GraphDefinedDomain absDG = phiMod.abstractMDP(dg, rf);
		RewardFunction rfA = phiMod.getRewardFunction();
		Domain absD= absDG.generateDomain();
		ValueIteration aVi = new ValueIteration(absD, rfA, tf, VIParams.gamma, hfA, VIParams.maxDelta, VIParams.maxIterations);
		State aInitialState = GraphDefinedDomain.getState(absD, 0);
		aVi.planFromState(aInitialState);	
		System.out.println("Abstract initial state value: " + aVi.value(aInitialState));
		
		// TESTING STUFF
		GroundedAction ga = absD.getActions().get(0).getAssociatedGroundedAction();

	}



}