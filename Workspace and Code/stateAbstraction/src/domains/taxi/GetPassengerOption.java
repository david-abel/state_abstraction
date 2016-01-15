package domains.taxi;

import java.util.List;

import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.options.DeterministicTerminationOption;
import burlap.oomdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.State;


public class GetPassengerOption extends DeterministicTerminationOption {

	public GetPassengerOption( Policy p, StateConditionTest init) {
		super("getPassengerOption", p, init, new IsPassengerInCarSCT());
		// TODO Auto-generated constructor stub
	}

	public static boolean isTaxiEmpty(State s) {
		List<ObjectInstance> allPassengers = s.getObjectsOfClass(TaxiDomainGenerator.TAXICLASS);
	
		for(ObjectInstance passenger : allPassengers) {
			boolean inTaxi = passenger.getBooleanValForAttribute(TaxiDomainGenerator.BEENPICKEDUPATT);
			
			if (inTaxi) {
				// There's a passenger in a car.
				return false;
			}
		}
		return true;
	}


}



// Initiation condition.
class IsTaxiEmptySCT implements StateConditionTest {
	@Override
	public boolean satisfies(State s) {
		return !GetPassengerOption.isTaxiEmpty(s);
	}
}

// Terminal condition.
class IsPassengerInCarSCT implements StateConditionTest {

	@Override
	public boolean satisfies(State s) {
		return GetPassengerOption.isTaxiEmpty(s);
	}
	
}