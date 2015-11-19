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
	 * 
	 * @param totalNumStates
	 * @return an n state graph MDP.
	 */
	public static GraphDefinedDomain getNStateChain(int totalNumStates) {
		double slipProb = .2;
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
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 0/*action*/, 0,1.0-slipProb);
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 1/*action*/, totalNumStates-1, 1.0-slipProb);
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 1/*action*/, 0, slipProb);

		return dg;
	}	
	
	/**
	 * n State chain reward function -- returns 10 if action 1 is taken in the last state, 2 if
	 * action 0 is taken ever and 0 otherwise.
	 * @author Dhershkowitz
	 *
	 */
	public static class nStateChainRF implements RewardFunction {
		int n;
		
		public nStateChainRF(int n) {
			this.n = n;
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			if (a.actionName().equals("action0")) {
				return 2;
			}
			else {
				if (GraphDefinedDomain.getNodeId(s) == n-1 && GraphDefinedDomain.getNodeId(sprime) == n-1) {
					return 10;
				}
			}
			return 0;
		}
		
	}

	/**
	 * An example of phiModel (currently not actually implemented).
	 * @param args
	 */
	public static void main(String[] args) {
		//VI Params
		int maxIterations = 10000;
		double maxDelta = .0001;
		double gamma = .95;
		TerminalFunction tf = new NullTermination();
		
		// Ground MDP
		int n = 5;
		HashableStateFactory hf = new SimpleHashableStateFactory();
		GraphDefinedDomain dg = getNStateChain(n);
		RewardFunction rf = new nStateChainRF(n);
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
		
		// TESTING STUFF
		GroundedAction ga = absD.getActions().get(0).getAssociatedGroundedAction();

	}



}