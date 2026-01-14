class RobotScoringPositions:
	elevator_raise_threshold = 1.0 # meters needed to be closer than to reef to raise elevator
	end_effector_travel_position = 8.1 # 8.0
	elevator_intake_height = 2.0
	end_effector_intake_position = 0.0
	min_elevator_height_to_bring_in_end_effector = 4.0
	min_end_effector_position_to_move_elevator_up = 7.5
	dangerous_tip_angle = 15.0
	end_effector_climbing_position = 10.0
	elevator_climb_height = 13.0
	min_end_effector_position_to_climb = 7.7
	min_elevator_climb_height = 10.0
	climber_up_position = 0.5396
	climber_climb_position = 0.1
	climber_default_position = 0.0
	piece_touching_L1_rim_torque_current = 15.0
	L1_elevator_raise_height = 8.0
	class L1_Scoring:
		elevator_height = 10.0
		end_effector_outtake_speed = 12.0
		end_effector_position = 10.0
		number = 1
	class L2_Scoring:
		elevator_height = 23.5
		end_effector_outtake_speed = 30.0 #35.0
		end_effector_position = 10.0
		number = 2
	class L2_Scoring_Blocked:
		elevator_height = 30.0
		end_effector_outtake_speed = 25.0
		end_effector_position = 10.0
		number = 2
	class L3_Scoring:
		elevator_height = 39.5 # 39.0
		end_effector_outtake_speed = 30.0 #35.0 # 30.0
		end_effector_position = 10.0
		number = 3
	class L3_Scoring_Blocked:
		elevator_height = 50.0
		end_effector_outtake_speed = 20.75
		end_effector_position = 10.0
		number = 3
	class L4_Scoring:
		elevator_height = 61.5
		end_effector_outtake_speed = 42.0 #42.0 #65.0
		end_effector_position = 10.0
		number = 4
	class Descore_Algae_L3:
		elevator_height = 37.5
		end_effector_outtake_speed = 40
		end_effector_position = 10.0
		number = 6
	class Descore_Algae_L2:
		elevator_height = 20.0
		end_effector_outtake_speed = 40
		end_effector_position = 10.0
		number = 5