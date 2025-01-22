/**
* Name: TSP
* Based on the internal empty template. 
* Author: hnv
* Tags: 
*/


model TSP

global {
	//Generall Variable
	string scenario; 
	int neigborhood_type;
	file Map <- csv_file("../includes/" + scenario + ".csv");
	matrix Map_matrix <- matrix(Map);
	int Map_width <- Map_matrix.columns();
	int Map_height <- Map_matrix.rows();
	
	// Date and Time varibale
	float step <- 10 #s;
	date my_date <- date("2025-1-1T00:00:00+07:00"); 
	
	// General size of species (car, robot, charging poles) in this simulation
	float general_size <- 2.0;
	
	// Robot's variable
	point robot_location;
	point robot_initial_location;
	list<point> list_goals;
	list<point> list_goal_in_optimal_sequence;
	path robot_path;
	list<path> robot_total_path;
	
	
	// Car's variable
	list<image_file> ev_car_icons <- [
		file("../includes/images/electric-car-green.png"),
		file("../includes/images/electric-car-red.png")
	];
	point car_parking_location;
	// This variable is used for storing the group of car position created in 10 cycle
	list car_group_location;
	// This variable is used to store list of car species in 10 cycle;
	list car_group;
	// This variable is used to track how many car need charge;
	list<car> car_group_need_charge;
	float car_generate_possibility <- 0.1;
	float car_charging_possibility <- 0.5;
	// This variable is used for creating the first position of car when entering basement
	list car_initial_locations_list <- [];
	list<path> list_car_path_moving_in;
	list<path> list_car_path_moving_out;
	// This variable is used to store location of car that need charge
	list<point> list_car_need_charge_locations;
	
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
		robot_location <- point((one_of (cell where (not each.is_obstacle and not each.is_parking_zone))).location);
		robot_initial_location <- robot_location;
		create robot number: 1;
	}
	
	reflex update_color_for_map {
		ask cell {
			if (is_obstacle) {
				color <- #black;
			} else if (is_occupied) {
				color <- #red;
			} else if (is_parking_zone) {
				color <- #yellow;
			}
		}
	}
	
	reflex car_generated when: every(1#mn) {
		bool is_car_generated <- flip(car_generate_possibility);
		if (is_car_generated) {
			cell parking_cell <- one_of (cell where (each.is_parking_zone and not each.is_occupied));
			car_parking_location <- point(parking_cell.location);
			parking_cell.is_occupied <- true;
			ask parking_cell {
				if (is_occupied) {
					color <- #red;
				}
			}
			create car number: 1; 
			car created_car <- car.population[length(car.population)- 1];
			
			//Update on 2 groups of car location and car species
			car_group_location <+ car_parking_location;
			
			bool is_car_need_charge <- flip(car_charging_possibility);
			// If car need charge the icon will be red car and if car does not need charge then the icon will be green car
			if (is_car_need_charge) {
				ask created_car {
					// Features
					need_charged <- true;
					car_target_location <- car_parking_location;
					
					//Actions
					do get_car_icon;
					do move_to_parking_lot;
					
					write ("There are " + length(list_car_need_charge_locations) + " in the line!" );
//						write("These are cars need charge: " + car_group_need_charge);
				}
			} else {
				ask created_car {
					//Features
					need_charged <- false;
					car_target_location <- car_parking_location;
					
					//Actions
					do get_car_icon;
					do move_to_parking_lot;
				}
			}
		}
	}
	
	reflex robot_move_to_charge_car when: every(1#hour) {
		list_goal_in_optimal_sequence <- [];
		robot_total_path <- [];
		if (length(list_car_need_charge_locations) > 0) {
			ask robot {
				path_combined <- [];
				robot_location <- location;
				list_goal_in_optimal_sequence <- tsp_solver(robot_location, list_car_need_charge_locations);
				write("This is the optimal sequence: " + list_goal_in_optimal_sequence);
				loop i from: 0 to: (length(list_goal_in_optimal_sequence) - 2){
					point source <- list_goal_in_optimal_sequence[i];
					point goal <- list_goal_in_optimal_sequence[i+1]; 
					using topology(cell) {
						robot_path <- path_between((cell where not each.is_obstacle), source, goal);
					}
					robot_total_path <+ robot_path;
					write("Optimal Path ID: " + length(robot_total_path) + "Path: " + robot_path.vertices);
				}
				start_moving_to_charge <- true;
				robot_cycle <- 0;
				do combine_path;
			}
			//Reset all the tracking variable
			list_car_need_charge_locations <- [];
			car_group_location <- [];
			car_group <- car.population;
//			do pause;	
		}
	}
	
//	reflex testing_robot_movement {
//		loop i from: 0 to: 3 {
//			list_goals <+ point((one_of (cell where each.is_parking_zone)).location);
//		}
//		ask robot {
//			robot_path <- tsp_solver(robot_location, list_goals);
//			write("Optimal path found: " + robot_path);
//			list_goals <- [];
//		}
//		do pause;
//	}
}

species robot {
	float size <- general_size;
	rgb color <- #blue;
	image_file robot_icon <- image_file("../includes/images/robot.png");
	bool start_moving_to_charge <- false;
	int robot_cycle <- 0;
	list<point> path_combined;
	
	init {
		location <- robot_location;
	}
	
	//Calculate the Euclidean distance between two points.
	float calculate_distance(point point1, point point2){
		float Euclidean_distance;
		Euclidean_distance <- point1 distance_to point2;
//		write("Distance between " + point1 + " to " + point2 + " is " + Euclidean_distance);
		return Euclidean_distance;
	}
	
	//Generate all permutations of a list of points directly using recursion and swapping.
	list generate_permutations(list<point> points) {
		list<list> result <- [];
		list<int> indices_stack <- [0];
		list<list> points_stack <- [points];
		int start_index;
		list<point> current_points;
		
		write("This is points stack: " + points_stack);
		
		loop while: (length(indices_stack) > 0) {
			// The same concept with stack however can not found stack on GAMA so using list
//			write("Before: Indices_stack length: " + length(indices_stack) + " and Points_stack length: " + length(points_stack));
			start_index <- indices_stack[length(indices_stack ) - 1];
			remove last(start_index) from: indices_stack;
			current_points <- points_stack[length(points_stack) - 1];
//			write("Current points is " + current_points);
			remove last([points_stack[length(points_stack) - 1]]) from: points_stack;
//			write("After: Indices_stack length: " + length(indices_stack) + " and Points_stack length: " + length(points_stack));
			//write("Compare " + start_index + " and " + length(current_points));
			if (start_index = length(current_points)) {
				result <<+ [current_points];
			} else {
				loop i from: start_index to: length(current_points) -1 {
					point temp <- current_points[start_index];
					current_points[start_index] <- current_points[i];
					current_points[i] <- temp;
					
					indices_stack <+ start_index + 1;
					points_stack <<+ [current_points];
					
//					temp <- current_points[start_index];
//					current_points[start_index] <- current_points[i];
//					current_points[i] <- temp;
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
			
//			write("This is checking permutation" + perm);
			
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
//				write("End of one perm");
			}
			
			if(path_checking_length < min_path_length) {
				min_path_length <- path_checking_length;
				optimal_path <- path_checking;
			}	
		}
		
		return optimal_path;
	}
	
	action combine_path {
		loop p over: robot_total_path {
			path_combined <<+ p.vertices;
			write("This is combined path" + path_combined);
		}
	}
	
	// Moving action
//	action move_to_charge {
//		loop current_path over: robot_total_path {
//			loop i from:0 to: (length(current_path.vertices) - 1) {
//				location <- current_path.vertices[i];
//			}
//		}
//	}
	 reflex move_to_charge when:(start_moving_to_charge and robot_cycle < length(path_combined)) {
	 	location <- path_combined[robot_cycle];
	 	robot_cycle <- robot_cycle + 1;
	 	if (list_goal_in_optimal_sequence contains location) {
	 		ask car {
	 			if (car_target_location = myself.location) {
	 				do move_out_of_parking_lot;
	 			}
	 		}
	 	}
	 }
	
	aspect icon{
		draw robot_icon size: size;
	}
}

species car {
	bool need_charged;
	image_file car_icon;
	point car_initial_location <- point(one_of([cell[1,39], cell[2,39], cell[3,39]]));
	point car_target_location;
	bool reach_parking_location <- false;
	bool start_leaving_parking_location <- false;
	bool leave_parking_location <- false;
	bool waiting_status <- false;
	path car_path_moving_in;
	path car_path_moving_out;
	int cycle_track_for_car_movement_in <- 0;
	int cycle_track_for_car_movement_out <- 0;
	int leaving_location_counter <- 0;
	int car_fully_charged_cycle <- 0;
	int car_waiting_time <- rnd(360, 720, 5);
	
	init {
		location <- car_initial_location;
	}
	
	action get_car_icon{
		if(need_charged) {
			car_icon <- ev_car_icons[1];
		} else {
			car_icon <- ev_car_icons[0];
		}
	}
	
	action move_to_parking_lot {
		using topology(cell) {
			car_path_moving_in <- path_between((cell where (not each.is_obstacle)), car_initial_location, car_target_location);
		}
		list_car_path_moving_in <+ car_path_moving_in;
	}
	
	action waiting_to_be_charged {
		write(self.name + " is ready to be charged");
		list_car_path_moving_in >- car_path_moving_in;
		waiting_status <- true;
		if(need_charged) {
			do adding_on_car_charging_list;
		}
	}
	
	action move_out_of_parking_lot {
		using topology(cell) {
			car_path_moving_out <- path_between((cell where (not each.is_obstacle)), car_target_location, car_initial_location);
		}
		list_car_path_moving_out <+ car_path_moving_out;
		start_leaving_parking_location <- true;
		write("car Target location in real: " + car_target_location);
		ask cell(car_target_location) {
			is_occupied <- false;
			write("car Target location in cell: " + location);
		}
		
	}
	
	action adding_on_car_charging_list {
		car_group_need_charge <+ self;
		list_car_need_charge_locations <+ self.location;
	}
	
	reflex move_toward_parking_lot when: (not reach_parking_location and not start_leaving_parking_location) {
		location <- car_path_moving_in.vertices[cycle_track_for_car_movement_in];
		cycle_track_for_car_movement_in <- cycle_track_for_car_movement_in + 1;
	}
	
	reflex check_if_in_target_location {
		reach_parking_location <- (location = car_target_location);
//		write("Car has reach parking location ? " + reach_parking_location);
		// Add waiting_status to ensure the function waiting to be charged will not be called multiple time
		if (reach_parking_location and not waiting_status) {
			do waiting_to_be_charged;
		}
	}
	
	reflex move_out_of_parking_lot when: (start_leaving_parking_location) {
		location <- car_path_moving_out.vertices[cycle_track_for_car_movement_out];
		cycle_track_for_car_movement_out <- cycle_track_for_car_movement_out + 1;
	}
	
	reflex check_if_has_moved_out {
		leave_parking_location <- (location = car_initial_location);
//		write("Car has move out parking location ? " + leave_parking_location);
		if (leave_parking_location) {
			if (leaving_location_counter = 1) {
				do die;
			} else {
				leaving_location_counter <- leaving_location_counter + 1;
			}
		}
	}
	
	reflex fully_charged_car_moving_out {
		if (not need_charged) {
			if (reach_parking_location) {
				if (car_fully_charged_cycle = car_waiting_time) {
					do move_out_of_parking_lot;
				} else {
					car_fully_charged_cycle <- car_fully_charged_cycle + 1;
				}
			}
		}
	}
	
	aspect icon {
		draw car_icon size: general_size;
	}
}

grid cell width: Map_width height: Map_height neighbors: neigborhood_type {
	bool is_obstacle;
	bool is_parking_zone;
	bool is_occupied;
} 
 

experiment TSP type: gui {
	parameter "MAP: " var: scenario <- "parking_lot" among: ["parking_lot"];
	parameter "Type of Neighborhood: " var: neigborhood_type <- 4 among: [4, 8];
	parameter "Car Generate Posibility: " var: car_generate_possibility min: 0.01 max: 0.4; 
	parameter "Car Need Charging Posibility: " var: car_charging_possibility min: 0.01 max: 0.6; 
	
	output synchronized: true {
		display main_display type: 2d antialias: false {
			grid cell border: #black;
			graphics "elements" {
				// Draw car path moving in
				if (length(list_car_path_moving_in) > 0){
					loop cp over: list_car_path_moving_in {
						loop v over: cp.vertices[0::(length(cp.vertices)-1)] {
							draw triangle(0.5) color: #yellow border: #red at: point(v);
						}
						loop s over: cp.segments {
							draw s color: #red ;
						}
					}	
				}
				// Draw car path moving out 
				if (length(list_car_path_moving_out) > 0) {
					loop cp over: list_car_path_moving_out {
						loop v over: cp.vertices[0::(length(cp.vertices)-1)] {
							draw triangle(0.5) color: #orange border: #red at: point(v);
						}
						loop s over: cp.segments {
							draw s color: #black ;
						}
					}
				}
				// Draw robot path going to charge car
				if (length(robot_total_path) > 0) {
					loop r over: robot_total_path {
						loop i over: r.vertices[1::(length(r.vertices) - 1)]{
							draw triangle(0.5) color: #pink border: #black at: point(i);
						}
						loop s over: r.segments {
							draw s color: #green ;
						}
					}
				}
			}
			species robot aspect: icon;
			species car aspect: icon;
		}
	}
}
