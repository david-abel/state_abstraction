package domains.trench;

import java.util.List;

import domains.GraphRF;
import domains.GraphTF;
import domains.NormalDomainToGraphDomain;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectClass;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.objects.MutableObjectInstance;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.MutableState;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.FullActionModel;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.SimpleAction.SimpleDeterministicAction;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;


public class TrenchDomainGenerator implements DomainGenerator {
	
	// OO-MDP attributes.
	public static final String ATTX = "x";
	public static final String ATTY = "y";
	public static final String ATTNUMBLOCKS = "numBlocks";

	// OO-MDP object classes.
	public static final String CLASSAGENT = "agent";
	public static final String CLASSCELL = "cell";

	// Action names.
	public static final String ACTIONNORTH = "north";
	public static final String ACTIONSOUTH = "south";
	public static final String ACTIONEAST = "east";
	public static final String ACTIONWEST = "west";
	public static final String ACTIONPLACE = "place";

	// Propositional functions.
	public static final String PFAT = "at";
	
	// Misc. parameters.
	private int maxBlockStackHeight = 2;
	int width;
	int height;
	int trenchY;
	
	public TrenchDomainGenerator(int width, int height) {
		this.width = width;
		this.height = height;
		this.trenchY = height / 2;
	}
	
	
	@Override
	public Domain generateDomain() {

		SADomain domain = new SADomain();
		
		// Make attributes
		Attribute xAtt = new Attribute(domain, ATTX, Attribute.AttributeType.INT);
		xAtt.setLims(0, this.width);

		Attribute yAtt = new Attribute(domain, ATTY, Attribute.AttributeType.INT);
		yAtt.setLims(0, this.height);
		
		Attribute numBlocksAtt = new Attribute(domain, ATTNUMBLOCKS, Attribute.AttributeType.INT);
		numBlocksAtt.setLims(0, this.maxBlockStackHeight);
		
		// Make object classes.
		ObjectClass agentClass = new ObjectClass(domain, CLASSAGENT);
		agentClass.addAttribute(xAtt);
		agentClass.addAttribute(yAtt);

		ObjectClass cellClass = new ObjectClass(domain, CLASSCELL);
		cellClass.addAttribute(xAtt);
		cellClass.addAttribute(yAtt);
		cellClass.addAttribute(numBlocksAtt);
		
		// Add actions.
		new Movement(ACTIONNORTH, domain, 0, this.width, this.height);
		new Movement(ACTIONSOUTH, domain, 1, this.width, this.height);
		new Movement(ACTIONEAST, domain, 2, this.width, this.height);
		new Movement(ACTIONWEST, domain, 3, this.width, this.height);
		new PlaceBlock(ACTIONPLACE, domain, this.width, this.height, this.maxBlockStackHeight);
		
		return domain;
	}
	
	/**
	 * Retrieves the cell object located at @x, @y, in state @s.
	 * @param s
	 * @param x
	 * @param y
	 * @return
	 */
	public static ObjectInstance getCellAt(State s, int x, int y) {
		List<ObjectInstance> allCells = s.getObjectsOfClass(CLASSCELL);
		
		for(ObjectInstance cell : allCells) {
			int cellX = cell.getIntValForAttribute(ATTX);
			int cellY = cell.getIntValForAttribute(ATTY);
			
			if (cellX == x && cellY == y) {
				return cell;
			}
		}
		return null;
	}
	
	/**
	 * Retrieves the initial state of the Trench domain.
	 * @param domain
	 * @return
	 */
	public State getInitialState(Domain domain) {
		
		// Set the initial conditions for the agent.
		State s = new MutableState();
		ObjectInstance agent = new MutableObjectInstance(domain.getObjectClass(CLASSAGENT), "agent0");
		agent.setValue(ATTX, 0);
		agent.setValue(ATTY, 0);
		s.addObject(agent);

		for(int x = 0; x < this.width; x++) {
			for(int y = 0; y < this.height; y++) {
			
				ObjectInstance newCell = new MutableObjectInstance(domain.getObjectClass(CLASSCELL), "cell" + x + "," + y);
				newCell.setValue(ATTX, x);
				newCell.setValue(ATTY, y);
				
				if(y == this.trenchY) {
					// We're at the trench, don't add a block.
					newCell.setValue(ATTNUMBLOCKS, 0);
				}
				else{
					// We're not at the trench, add a block to the cell.
					newCell.setValue(ATTNUMBLOCKS, 1);
				}
				
				s.addObject(newCell);
				
			}
		}

		return s;
	}
	
	/**
	 * Agent can place blocks only to the North. 
	 * @author dabel
	 *
	 */
	protected class PlaceBlock extends SimpleDeterministicAction implements FullActionModel {

		int width;
		int height;
		int maxNumBlocks;
		int trenchY;
		
		public PlaceBlock(String actionName, SADomain domain, int width, int height, int maxNumBlocks) {
			super(actionName, domain);
			this.width = width;
			this.height = height;
			this.maxNumBlocks = maxNumBlocks;
			
			this.trenchY = height / 2;
		}


		@Override
		protected State performActionHelper(State s, GroundedAction groundedAction) {
			
			// Get agent and current position.
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curX = agent.getIntValForAttribute(ATTX);
			int curY = agent.getIntValForAttribute(ATTY);
			
			if (curY >= this.height - 1) {
				// If the agent is at the top, can't place.
				return s;
			}
			
			// Place the block in front.
			ObjectInstance cellInFrontOfAgent = getCellAt(s, curX, curY + 1);
			int numBlocksInFront = cellInFrontOfAgent.getIntValForAttribute(ATTNUMBLOCKS);
			
			if (numBlocksInFront >= maxNumBlocks) {
				return s;
			}
			
			cellInFrontOfAgent.setValue(ATTNUMBLOCKS, numBlocksInFront + 1);
			
			return s;
		}



	}
	
	/**
	 * One class for North, East, South, and West (parameter direction determines which one).
	 * @author dabel
	 * direction: 0=North, 1=South, 2=East, 3=West.
	 */
	protected class Movement extends SimpleDeterministicAction implements FullActionModel {

		int direction;
		int width;
		int height;
		
		public Movement(String actionName, Domain domain, int direction, int width, int height){
			super(actionName, domain);
			this.direction = direction;
			this.width = width;
			this.height = height;
		}

		@Override
		protected State performActionHelper(State s, GroundedAction groundedAction) {
			
			// Get agent and current position.
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curX = agent.getIntValForAttribute(ATTX);
			int curY = agent.getIntValForAttribute(ATTY);

			// Get resulting position.
			int[] newPos = this.moveResult(s, curX, curY, this.direction);
			//set the new position
			agent.setValue(ATTX, newPos[0]);
			agent.setValue(ATTY, newPos[1]);

			// Return the state we just modified.
			return s;
		}


		protected int[] moveResult(State s, int curX, int curY, int direction){

			// First get change in x and y from direction using 0: north; 1: south; 2:east; 3: west
			int xdelta = 0;
			int ydelta = 0;
			if(direction == 0){
				// MOVE NORTH
				ydelta = 1;
			}
			else if(direction == 1){
				// MOVE SOUTH
				ydelta = -1;
			}
			else if(direction == 2){
				// MOVE EAST
				xdelta = 1;
			}
			else{
				// MOVE WEST
				xdelta = -1;
			}

			int nx = curX + xdelta;
			int ny = curY + ydelta;

			// Make sure new position is valid (not a wall or off bounds)
			if(nx < 0 || nx >= this.width || ny < 0 || ny >= this.height || !canWalkOnCell(s, curX, curY, nx, ny)){
				nx = curX;
				ny = curY;
			}

			return new int[]{nx,ny};
		}

	}
	
	/**
	 * Determines if an agent can move from (@oldX,@oldY) to (@newX,@newY) in state @s.
	 * @param s
	 * @param oldX
	 * @param oldY
	 * @param newX
	 * @param newY
	 * @return
	 */
	private boolean canWalkOnCell(State s, int oldX, int oldY, int newX, int newY) {
		ObjectInstance oldCell = getCellAt(s, oldX, oldY);
		ObjectInstance newCell = getCellAt(s, newX, newY);
		
		if (oldCell == null || newCell == null) {
			return false;
		}
		
		int numOldBlocks = oldCell.getIntValForAttribute(ATTNUMBLOCKS);
		int numNewBlocks = newCell.getIntValForAttribute(ATTNUMBLOCKS);
		
		
		if (numOldBlocks >= numNewBlocks) {
			// If we are "higher up", we can move to the lower ground. 
			return true;
		}
		else {
			// The new block is higher than us.
			return false;
		}
	}

	/**
	 * Trench Reward: 1 at goal location, 0 everywhere else.
	 * @author dabel
	 *
	 */
	public static class TrenchRF implements RewardFunction {

		int goalX;
		int goalY;

		public TrenchRF(int goalX, int goalY) {
			this.goalX = goalX;
			this.goalY = goalY;
		}
		
		@Override
		public double reward(State s, GroundedAction a, State sprime) {

			// Get location of agent in next state.
			ObjectInstance agent = sprime.getFirstObjectOfClass(CLASSAGENT);
			int ax = agent.getIntValForAttribute(ATTX);
			int ay = agent.getIntValForAttribute(ATTY);

			// Are they at goal location?
			if(ax == this.goalX && ay == this.goalY){
				return 1;
			}

			return 0;
		}
	}

	/**
	 * Trench Terminal Function. Terminal iff agent at goal location. 
	 * @author dabel
	 *
	 */
	public static class TrenchTF implements TerminalFunction {
		
		int goalX;
		int goalY;

		public TrenchTF(int goalX, int goalY) {
			this.goalX = goalX;
			this.goalY = goalY;
		}
		
		@Override
		public boolean isTerminal(State s) {
			// Get location of agent in next state.
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int ax = agent.getIntValForAttribute(ATTX);
			int ay = agent.getIntValForAttribute(ATTY);

			// Are they at goal location?
			if(ax == this.goalX && ay == this.goalY){
				return true;
			}

			return false;
		}
			
	}
	
	
	public static void main(String [] args){
		
		//----TEST TRENCH DOMAIN----

		// Setup the domain.
		int height = 3;
		int width = 3;
		TrenchDomainGenerator gen = new TrenchDomainGenerator(height, width);
		Domain domain = gen.generateDomain();
		State initialState = gen.getInitialState(domain);

		// Set reward and terminal functions.
		RewardFunction rf = new TrenchRF(width - 1, height - 1);
		TerminalFunction tf = new TrenchTF(width - 1, height - 1);

		// Set planning parameters.
		double gamma = 0.99;
		int numRollouts = 100;
		double minDelta = 0.8;
		
		// Create VI and plan.
		ValueIteration vi = new ValueIteration(domain, rf, tf, gamma, new SimpleHashableStateFactory(), minDelta, numRollouts);
		GreedyQPolicy policy = vi.planFromState(initialState);
		EpisodeAnalysis ea = policy.evaluateBehavior(initialState, rf, tf);
		System.out.println("NonGraph reward sequence: " + ea.rewardSequence);

		
		// Test converting it into a graph and planning.
		NormalDomainToGraphDomain graphTrenchMaker = new NormalDomainToGraphDomain(gen, tf, rf, initialState);
		GraphDefinedDomain trenchGraphDefinedDomain = graphTrenchMaker.createGraphDomain();
		Domain graphDomain = trenchGraphDefinedDomain.generateDomain();
		State graphInitState = GraphDefinedDomain.getState(graphDomain, graphTrenchMaker.initStateID);
		RewardFunction graphRF = new GraphRF(graphDomain, rf, tf, graphTrenchMaker.graphIndexToNonGraphState, graphTrenchMaker.graphActionIndexToNonGraphAction);
		TerminalFunction graphTF = new NullTermination();
		ValueIteration vi2 = new ValueIteration(graphDomain, graphRF, graphTF, gamma, new SimpleHashableStateFactory(), minDelta, numRollouts);
		GreedyQPolicy policy2 = vi2.planFromState(graphInitState);
		EpisodeAnalysis ea2 = policy2.evaluateBehavior(graphInitState, graphRF, graphTF, 100);
		System.out.println("Graph reward sequence:" + ea2.rewardSequence);
	}
	
}
