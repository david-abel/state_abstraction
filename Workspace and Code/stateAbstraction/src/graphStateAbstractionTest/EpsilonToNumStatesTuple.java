package graphStateAbstractionTest;

public class EpsilonToNumStatesTuple {
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
