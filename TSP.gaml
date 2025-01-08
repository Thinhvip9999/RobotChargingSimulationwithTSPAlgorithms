/**
* Name: TSP
* Based on the internal empty template. 
* Author: hnv
* Tags: 
*/


model TSP

global {
	//Generall parameter
	string scenario; 
	int neigborhood_type;
	int grid_size_height;
	int grid_size_width;
	
	reflex check {
		
	}
}

species robot {
	//Calculate the Euclidean distance between two points.
	float calculate_distance(point point1, point point2){
		float Euclidean_distance;
		Euclidean_distance <- point1 distance_to point2;
		return Euclidean_distance;
	}
	
	//Generate all permutations of a list of points directly using recursion and swapping.
	list generate_permutations(list<point> points) {
		list result <- [];
		return result;
	}
	
	//Solve the Traveling Salesman Problem (TSP).
	path tsp_solver (point current_position, list<point> points) {
		path robot_path;
		return robot_path;
	}
}
 

experiment tspExperiment type: gui {
	parameter "MAP:" var: scenario <- "basement map" among: ["basement map"];
	parameter "Type of Neighborhood:" var: neigborhood_type <- 4 among: [4, 8];
}
