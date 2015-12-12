package SARealGenerators;

import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;

/**
 * Just a wrapper class for keying into hash tables based on state, action pairs.
 *  Or better yet, a rapper class -- SAPair in the house.
 * @author Dhershkowitz
 *
 */
public class SAPair {
	public State s;
	public Action a;

	/**
	 * 
	 * @param s
	 * @param a
	 */
	public SAPair(State s, Action a) {
		this.s = s;
		this.a = a;
	}
	
	@Override
	public boolean equals(Object other) {
		if (other instanceof SAPair) {
			SAPair otherSA = ((SAPair) other);
			return otherSA.a.equals(a) && otherSA.s.equals(s);
		}
		return false;
	}
	
	
	@Override
	public int hashCode() {
		return s.toString().hashCode()+a.getAssociatedGroundedAction().actionName().hashCode();
	}
	
	@Override
	public String toString() {
		return "(" + s.toString() + ", " + a.getAssociatedGroundedAction().actionName() + ")";
	}

}
