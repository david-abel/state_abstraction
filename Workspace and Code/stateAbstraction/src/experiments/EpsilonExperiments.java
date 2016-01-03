package experiments;

import graphStateAbstractionTest.QStarEpsilonTest;
import graphStateAbstractionTest.QStarEpsilonTest.EpsilonToNumStatesTuple;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import burlap.debugtools.DPrint;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.RewardFunction;
import domains.GraphRF;
import domains.GraphTF;
import domains.NormalDomainToGraphDomain;
import domains.nchain.NChainGenerator;
import domains.taxi.TaxiDomainGenerator;
import domains.trench.TrenchDomainGenerator;
import domains.upworld.UpWorldGenerator;

/**
 * Code for running experiments for our ICML paper.
 * @author David Abel
 *
 */
public class EpsilonExperiments {
	
	static String filePath = new File("").getAbsolutePath();
	static String resultsDir = filePath + "/src/plotting/";
	
	// Iterate over epsilon and compute the number of states.
	final static double startEpsilon = 0.0;
	final static double endEpsilon = .1;
	final static double epsilonIncrement = 0.005;
	
	/**
	 * Given a DomainGenerator + RF + TF + initState, this method generates abstract MDPs subject to different epsilons, and prints the results.
	 * @param domainName
	 * @param gen
	 * @param tf
	 * @param rf
	 * @param initialState
	 */
	public static void generateEpsilonResults(GraphDefinedDomain graphDefinedDomain, TerminalFunction graphTF, RewardFunction graphRF, State initGraphState, String taskName) {
		
		List<EpsilonToNumStatesTuple> epsilonAndNumStatesPairs = QStarEpsilonTest.testQPhiStateReduction(graphDefinedDomain, graphRF, graphTF, initGraphState, startEpsilon, endEpsilon, epsilonIncrement);
		
		List<Double> epsilons = new ArrayList<Double>();
		List<Integer> numStates = new ArrayList<Integer>();
		
		clearOldResultsFile(taskName);
		
		System.out.println("results: ");
		for (EpsilonToNumStatesTuple x : epsilonAndNumStatesPairs) {
			epsilons.add(x.getEpsilon());
			numStates.add(x.getNumStates());
			writeDataPointToFile(x, taskName);
			System.out.println(x);
		}
		
		// Now make the plot...
	}
	
	private static void clearOldResultsFile(String taskName) {
		FileWriter fileOut;
		try {
			fileOut = new FileWriter(resultsDir + "/" + taskName + "/" + taskName + ".results", false);
			fileOut.write("");
			fileOut.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private static void writeDataPointToFile(EpsilonToNumStatesTuple x, String taskName) {
		try(PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(resultsDir + "/" + taskName + "/" + taskName + ".results", true)))) {
			out.println(x);  
		}
		catch (IOException e) {
		    //exception handling left as an exercise for the reader
			e.printStackTrace();
		}
	}
	
	public static void compressAndGenEpsilonResults(DomainGenerator gen, TerminalFunction tf, RewardFunction rf, State initialState, String taskName) {
		// Convert to graph domain.
		NormalDomainToGraphDomain graphMaker = new NormalDomainToGraphDomain(gen, tf, rf, initialState);
		GraphDefinedDomain graphDefinedDomain = graphMaker.createGraphDomain();
		Domain d = graphDefinedDomain.generateDomain();
		RewardFunction graphRF = new GraphRF(graphMaker.goalStateIDs);
		TerminalFunction graphTF = new GraphTF(graphMaker.goalStateIDs);
		State initGraphState = GraphDefinedDomain.getState(d, graphMaker.initStateID);
		
		generateEpsilonResults(graphDefinedDomain, graphTF, graphRF, initGraphState, taskName);
	}
	
	public static void main(String[] args) {

		DPrint.toggleUniversal(false);
		
		// Create trench domain.
		int height = 3;
		int width = 3;
		TrenchDomainGenerator trenchGen = new TrenchDomainGenerator(height, width);
		TerminalFunction trenchTF = new TrenchDomainGenerator.TrenchTF(height - 1, width - 1);
		RewardFunction trenchRF = new TrenchDomainGenerator.TrenchRF(height - 1, width - 1);
		Domain oldTrenchDomain = trenchGen.generateDomain();
		State initialTrenchState = trenchGen.getInitialState(oldTrenchDomain);
//		resultsFile = resultsFile + "trench/";
//		
		
		// Create taxi domain.
		TaxiDomainGenerator taxiGen = new TaxiDomainGenerator();
		TerminalFunction taxiTF = new TaxiDomainGenerator.TaxiTF();
		RewardFunction taxiRF = new TaxiDomainGenerator.TaxiRF();
		Domain oldTaxiDomain = taxiGen.generateDomain();
		State initialTaxiState = taxiGen.getInitialState(oldTaxiDomain);
		
		// Create upworld domain.
		int upWorldHeight = 10;
		int upWorldWidth = 4;
		GraphDefinedDomain upWorldGen = UpWorldGenerator.getUPWorld(upWorldWidth, upWorldHeight);
		Domain upWorldDomain = upWorldGen.generateDomain();
		State initialUpWorldState = GraphDefinedDomain.getState(upWorldDomain, 0);
		TerminalFunction upWorldTF = new NullTermination();
		RewardFunction upWorldRF = new UpWorldGenerator.UpWorldRF();
		
		
		// Create nchain domain.
		int numStates = 50;
		GraphDefinedDomain nChainGen = NChainGenerator.getNStateChain(numStates);
		Domain nChainDomain = nChainGen.generateDomain();
		State initialNChainState = GraphDefinedDomain.getState(nChainDomain, 0);
		TerminalFunction nChainTF = new NullTermination();
		RewardFunction nChainRF = new NChainGenerator.nStateChainRF(numStates);

		String task = "TAXI"; // NCHAIN, TRENCH, TAXI, UPWORLD		
		
		if (task == "ALL") {
			generateEpsilonResults(nChainGen, nChainTF, nChainRF, initialNChainState, "nchain");
			compressAndGenEpsilonResults(trenchGen, trenchTF, trenchRF, initialTrenchState, "trench");
			compressAndGenEpsilonResults(taxiGen, taxiTF, taxiRF, initialTaxiState, "taxi");
			generateEpsilonResults(upWorldGen, upWorldTF, upWorldRF, initialUpWorldState, "upworld");
		}
		if (task == "NCHAIN") {
			generateEpsilonResults(nChainGen, nChainTF, nChainRF, initialNChainState, "nchain");
		}
		else if (task == "TRENCH") {
			compressAndGenEpsilonResults(trenchGen, trenchTF, trenchRF, initialTrenchState, "trench");
		}
		else if (task == "TAXI") {
			compressAndGenEpsilonResults(taxiGen, taxiTF, taxiRF, initialTaxiState, "taxi");
		}
		else {
			generateEpsilonResults(upWorldGen, upWorldTF, upWorldRF, initialUpWorldState, "upworld");
		}
		
		makePlots();
	}
	
	private static void makePlots() {
		String[] cmd = {
		        "/bin/bash",
		        "-c",
		        "python plotter.py all"
		    };
		try {
			Runtime.getRuntime().exec(cmd, null, new File(filePath + "/src/plotting/"));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}