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
	file Map <- csv_file("../includes/" + scenario + ".csv");
	matrix Map_matrix <- matrix(Map);
	int Map_width <- Map_matrix.columns();
	int Map_height <- Map_matrix.rows();
	
	init initialize {
		loop i from: 0 to: Map_height -1 {
			loop j from: 0 to: Map_width -1 {
				if (int(Map_matrix[j, i]) = 2) {
					cell[j, i].is_parking_zone <- true;
				} else if (int(Map_matrix[j, i]) = 1) {
					cell[j, i].is_obstacle <- true;
				}
			}
		}
		ask cell {
			if (is_obstacle) {
				color <- #black;
			} else if (is_parking_zone) {
				color <- #yellow;
			}
		}
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
		list indices_stack <- [0];
		list points_stack <- [points];
		int start_index;
		list current_points;
		
		loop while: (length(indices_stack) > 0) {
			// The same concept with stack however can not found stack on GAMA so using list
			start_index <- indices_stack[length(indices_stack ) - 1];
			remove last(start_index) from: indices_stack;
			current_points <- points_stack[length(points_stack) - 1];
			remove last(current_points) from: points_stack;
			
			if (start_index = length(current_points)) {
				add current_points to: result;
			} else {
				loop i from: start_index to: length(current_points) {
					point temp <- current_points[start_index];
					current_points[start_index] <- current_points[i];
					current_points[i] <- temp;
				}
			}
		}
		return result;
	}
	
	//Solve the Traveling Salesman Problem (TSP).
	path tsp_solver (point current_position, list<point> points) {
		path robot_path;
		return robot_path;
	}
}

grid cell width: Map_width height: Map_height neighbors: neigborhood_type {
	bool is_obstacle;
	bool is_parking_zone;
} 
 

experiment TSP type: gui {
	parameter "MAP:" var: scenario <- "parking_lot" among: ["parking_lot"];
	parameter "Type of Neighborhood:" var: neigborhood_type <- 4 among: [4, 8];
	
	output synchronized: true {
		display main_display type: 2d antialias: false {
			grid cell border: #black;
		}
	}
}
