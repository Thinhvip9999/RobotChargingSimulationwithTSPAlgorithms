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
	
	// General size of species (car, robot, charging poles) in this simulation
	float general_size <- 2.0;
	
	// Robot's variable
	point robot_location;
	list<point> list_goals;
	list<point> robot_path;
	
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
		robot_location <- point((one_of (cell where not each.is_obstacle)).location);
		create robot number: 1;
	}
	
	reflex testing_robot_movement {
		loop i from: 0 to: 3 {
			list_goals <+ point((one_of (cell where each.is_parking_zone)).location);
		}
		ask robot {
			robot_path <- tsp_solver(robot_location, list_goals);
			write("Optimal path found: " + robot_path);
			list_goals <- [];
		}
		do pause;
	}
}

species robot {
	float size <- general_size;
	rgb color <- #blue;
	image_file robot_icon <- image_file("../includes/images/robot.png");
	
	init {
		location <- robot_location;
	}
	
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
		
		write("This is points stack: " + points_stack);
		
		loop while: (length(indices_stack) > 0) {
			// The same concept with stack however can not found stack on GAMA so using list
			write("Before: Indices_stack length: " + length(indices_stack) + " and Points_stack length: " + length(points_stack));
			start_index <- indices_stack[length(indices_stack ) - 1];
			remove last(start_index) from: indices_stack;
			current_points <- points_stack[length(points_stack) - 1];
			write("Current points is " + current_points);
			remove last([points_stack[length(points_stack) - 1]]) from: points_stack;
			write("After: Indices_stack length: " + length(indices_stack) + " and Points_stack length: " + length(points_stack));
			//write("Compare " + start_index + " and " + length(current_points));
			if (start_index = length(current_points)) {
				result <+ current_points;
			} else {
				loop i from: start_index to: length(current_points) -1 {
					point temp <- current_points[start_index];
					current_points[start_index] <- current_points[i];
					current_points[i] <- temp;
					
					indices_stack <+ start_index + 1;
					points_stack <<+ current_points;
					
					temp <- current_points[start_index];
					current_points[start_index] <- current_points[i];
					current_points[i] <- temp;
					indices_stack <+ start_index + 1;
				}
			}
		}
		
		return result;
	}
	
	//Solve the Traveling Salesman Problem (TSP).
	list tsp_solver (point current_position, list<point> points) {
		write("Robot starting position is " + current_position);
		write("List of goals are " + points);
		list all_points <- [];
		list<list> target_permutations <- [];
		float min_path_length <- 9999999999.99; //#max_float
		list optimal_path <- [];
		// <+ add single variable 
		// <<+ add entire list of that type
		all_points <+ current_position;
		all_points <<+ points;
		write("Before permutation: " + points);
		target_permutations <- generate_permutations(points);
		write("Number of permutations is " + length(target_permutations));
		
		loop perm over: target_permutations {
			list path_checking;
			float path_checking_length;
			float distance_between_two_points;
			
			path_checking <+ current_position;
			path_checking <<+ perm;
//			write("Perm: " + perm);
			path_checking_length <- float(0);
			
			loop i from: 0 to: (length(path_checking) - 2) {
				distance_between_two_points <- calculate_distance(path_checking[i], path_checking[i+1]);
				path_checking_length <- path_checking_length + distance_between_two_points;
				if (path_checking_length >= min_path_length) {
//					write("No need to continue checking on this perm");
					break;
				}
				write("End of one perm");
			}
			
			if(path_checking_length < min_path_length) {
				min_path_length <- path_checking_length;
				optimal_path <- path_checking;
			}	
		}
		
		return optimal_path;
	}
	
	aspect icon{
		draw robot_icon size: size;
	}
}

species car {
	
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
			species robot aspect: icon;
		}
	}
}
