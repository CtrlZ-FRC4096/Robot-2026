# import numpy as np
# import math
# import time
# # import matplotlib.pyplot as plt
# # from matplotlib.animation import FuncAnimation
# # from mpl_toolkits.mplot3d import Axes3D
# from sleipnir.optimization import Problem
# import sleipnir.autodiff as ad
# from lookup_table import LookupTableAll
# # import pandas as pd
# class SleipnirRobustOptimizer:
#     def __init__(self, shooter_pos=np.array([0.0, 0.0, 0.0]), shooter_vel=np.array([0.0, 0.0, 0.0]), min_v=0.0, max_v=25.0, min_angle_deg=0.0, max_angle_deg=85.0):
        
#         # Constants
#         self.gravity = 9.81
#         self.air_density = 1.204
#         self.mass = 0.216
#         self.diameter = 0.15
#         self.area = np.pi * (self.diameter / 2)**2
#         self.drag_coefficient = 0.5
        
#         self.magnus_enabled = True
#         self.spin = -0.3
#         self.lift_factor = 0.2
        
#         # Target
#         self.target_pos = np.array([4.625594, 4.034536, 1.83])
#         self.shooter_pos = shooter_pos
#         self.shooter_vel = shooter_vel
        
#         # Constraints
#         self.max_v = max_v
#         self.min_v = min_v
#         self.min_angle_rad = math.radians(min_angle_deg)
#         self.max_angle_rad = math.radians(max_angle_deg)
        
#         # Uncertainty
#         self.v_std = -0.5
#         self.theta_std = math.radians(2.0)
#         self.phi_std = math.radians(3.0)

#         # Opt config
#         self.steps = 5
        
#     def dynamics(self, state, spin_axis=None):
#         # State: [x, y, z, vx, vy, vz]
#         vx = state[3]
#         vy = state[4]
#         vz = state[5]
        
#         # Velocity magnitude
#         # Using ad.sqrt
#         v_sq = vx*vx + vy*vy + vz*vz
#         v_mag = ad.sqrt(v_sq)
        
#         # Drag
#         k_drag = -0.5 * self.air_density * self.drag_coefficient * self.area * v_mag
        
#         ax_drag = k_drag * vx / self.mass
#         ay_drag = k_drag * vy / self.mass
#         az_drag = k_drag * vz / self.mass
        
#         # Magnus
#         if spin_axis is None:
#             axis_x = 0.0
#             axis_y = -1.0
#             axis_z = 0.0
#         else:
#             axis_x = spin_axis[0]
#             axis_y = spin_axis[1]
#             axis_z = spin_axis[2]
        
#         # Cross product (axis x v)
#         cp_x = axis_y * vz - axis_z * vy 
#         cp_y = axis_z * vx - axis_x * vz 
#         cp_z = axis_x * vy - axis_y * vx 
        
#         cp_mag = ad.sqrt(cp_x*cp_x + cp_y*cp_y + cp_z*cp_z)
        
#         # Lift mag
#         cl = self.lift_factor * abs(self.spin)
        
#         # Force = 0.5 * rho * v^2 * A * Cl
#         # Dir = cp / cp_mag
#         c_lift_mass = (0.5 * self.air_density * self.area * cl) / self.mass
        
#         denom = cp_mag + 1e-9
        
#         ax_magnus = c_lift_mass * v_sq * (cp_x / denom)
#         ay_magnus = c_lift_mass * v_sq * (cp_y / denom)
#         az_magnus = c_lift_mass * v_sq * (cp_z / denom)
        
#         # Gravity
#         az_grav = -self.gravity
        
#         return [
#             vx, vy, vz,
#             ax_drag + ax_magnus,
#             ay_drag + ay_magnus,
#             az_drag + az_magnus + az_grav
#         ]

#     def run_rk2(self, state, dt, spin_axis=None):
#         # Heuns method (RK2) - 2 evaluations per step vs 4 for RK4
#         k1 = self.dynamics(state, spin_axis)
        
#         state_k2 = [s + k * dt for s, k in zip(state, k1)]
#         k2 = self.dynamics(state_k2, spin_axis)
        
#         new_state = []
#         for i in range(6):
#             val = state[i] + (dt / 2.0) * (k1[i] + k2[i])
#             new_state.append(val)
            
#         return new_state

#     def run_rk4(self, state, dt, spin_axis=None):
#         k1 = self.dynamics(state, spin_axis)
        
#         state_k2 = [s + k * dt / 2 for s, k in zip(state, k1)]
#         k2 = self.dynamics(state_k2, spin_axis)
        
#         state_k3 = [s + k * dt / 2 for s, k in zip(state, k2)]
#         k3 = self.dynamics(state_k3, spin_axis)
        
#         state_k4 = [s + k * dt for s, k in zip(state, k3)]
#         k4 = self.dynamics(state_k4, spin_axis)
        
#         new_state = []
#         for i in range(6):
#             val = state[i] + (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
#             new_state.append(val)
            
#         return new_state

#     def optimize(self):
#         # Strategy 1: Fast Sweep (Steps=2, Default Guess)
#         self.steps = 2
#         res = self._solve_problem(guess_type="default")

#         status_str = str(res['status'])
#         # If successful and accurate, return
#         if "SUCCESS" in status_str and res['cost'] <= 10.0:
#             return res

#         # Strategy 2: High Precision (Steps=20, High Lob Guess)
#         # We jump straight to High Lob guess because if default failed at steps=2,
#         # it likely needs a higher angle to clear the "drag wall" or avoid local minima.
#         res = self._solve_problem(guess_type="high_lob")
        
#         status_str = str(res['status'])
#         if "SUCCESS" in status_str and res['cost'] <= 10.0:
#              return res
        
#         # If needed we can increase the solver steps but it seems fine at 2
#         # self.steps = 20
#         # Strategy 3: High Precision (Steps=20, Max Energy/Lower Angle Guess)
#         res = self._solve_problem(guess_type="min_angle")
            
#         return res

#     def _solve_problem(self, guess_type="default", use_rk2=False, reduce_perturbations=False):
#         problem = Problem()
        
#         # Smart Initialization
#         dx = self.target_pos[0] - self.shooter_pos[0]
#         dy = self.target_pos[1] - self.shooter_pos[1]
#         dist = math.sqrt(dx*dx + dy*dy)
#         yaw_init = math.atan2(dy, dx)
        
#         # Decision Variables
#         v0 = problem.decision_variable()
#         theta = problem.decision_variable()
        
#         # -- Initial Guess Logic --
#         if guess_type == "high_lob":
#             # Guess high angle (e.g. 75 deg), moderate velocity
#             th_guess = min(math.radians(75.0), self.max_angle_rad)
#             v_guess = 10.0
#         elif guess_type == "min_angle":
#             # Guess minimum angle (more direct shot), max velocity
#             th_guess = self.min_angle_rad + 0.1 # slightly above min
#             v_guess = self.max_v
#         else: # default
#             # Bias towards 60 degrees for lobs
#             mid_angle = (self.min_angle_rad + self.max_angle_rad) / 2.0
#             lob_bias = math.radians(60.0) 
#             th_guess = max(self.min_angle_rad, min(lob_bias, self.max_angle_rad))
#             v_guess = max(self.min_v, min(10.0, self.max_v))

#         v0.set_value(v_guess)
#         theta.set_value(th_guess)
#         # -------------------------
        
#         phi = problem.decision_variable()
#         phi.set_value(yaw_init)
        
#         T_flight = problem.decision_variable()
#         # Estimate T based on guess
#         v_xy = v_guess * math.cos(th_guess)
#         t_guess = dist / (v_xy + 1e-9)
#         T_flight.set_value(t_guess)
        
#         # Bounds
#         problem.subject_to(v0 >= self.min_v)
#         problem.subject_to(v0 <= self.max_v)
        
#         problem.subject_to(theta >= self.min_angle_rad)
#         problem.subject_to(theta <= self.max_angle_rad)

#         # Yaw bounds (allow +/- 45 deg from direct line)
#         base_yaw = math.atan2(self.target_pos[1] - self.shooter_pos[1], self.target_pos[0] - self.shooter_pos[0])
#         problem.subject_to(phi >= base_yaw - 0.8)
#         problem.subject_to(phi <= base_yaw + 0.8)
        
#         problem.subject_to(T_flight >= 0.1)
#         problem.subject_to(T_flight <= 3.0)
        
#         dt = T_flight / self.steps
        
#         if reduce_perturbations:
#             # Fast Mode: Check primary axes only (positive direction)
#             # 4 Total Scenarios
#             perturbations = [
#                 (0.0, 0.0, 0.0),
#                 (self.v_std, 0.0, 0.0),
#                 (0.0, self.theta_std, 0.0),
#                 (0.0, 0.0, self.phi_std),
#             ]
#         else:
#             # Full Robust Mode: Check "Corners" of the uncertainty volume
#             # This captures compound errors (e.g. High Velocity + Low Angle + Left Drift)
#             # 1 Nominal + 4 Corners = 5 Total Scenarios
#             perturbations = [(0.0, 0.0, 0.0)]
#             for s_v in [self.v_std]:
#                 for s_th in [self.theta_std, -self.theta_std]:
#                     for s_ph in [self.phi_std, -self.phi_std]:
#                         perturbations.append((s_v, s_th, s_ph))
        
#         total_cost = ad.Variable(0.0)
        
#         for dv, dtheta, dphi in perturbations:
#             v_curr = v0 + dv
#             theta_curr = theta + dtheta
#             phi_curr = phi + dphi
            
#             vx = v_curr * ad.cos(theta_curr) * ad.cos(phi_curr) + self.shooter_vel[0]
#             vy = v_curr * ad.cos(theta_curr) * ad.sin(phi_curr) + self.shooter_vel[1]
#             vz = v_curr * ad.sin(theta_curr) + self.shooter_vel[2]
            
#             # Spin axis depends on Yaw (phi)
#             spin_axis = [ad.sin(phi_curr), -ad.cos(phi_curr), 0.0]

#             # Initial state
#             state = [
#                 ad.Variable(self.shooter_pos[0]), 
#                 ad.Variable(self.shooter_pos[1]), 
#                 ad.Variable(self.shooter_pos[2]),
#                 vx, vy, vz
#             ]
            
#                     # Simulate
#             for _ in range(self.steps):
#                 if use_rk2:
#                     state = self.run_rk2(state, dt, spin_axis)
#                 else:
#                     state = self.run_rk4(state, dt, spin_axis)
                
#             x_final = state[0]
#             y_final = state[1]
#             z_final = state[2]
#             vx_final = state[3]
#             vy_final = state[4]
#             vz_final = state[5]

#             if dv == 0 and dtheta == 0 and dphi == 0:
#                 problem.subject_to(z_final == self.target_pos[2])
#                 # Uncomment this if you want to fall into the hoop at -2m/s
#                 problem.subject_to(vz_final < -4) 
                
#             # Cost calc (Minimize X-Y error at Target Z Height)
#             # dt_corr: time diff to reach exact Z plane
#             dt_corr = (self.target_pos[2] - z_final) / (vz_final + 1e-9)
            
#             x_proj = x_final + vx_final * dt_corr
#             y_proj = y_final + vy_final * dt_corr
            
#             dx = x_proj - self.target_pos[0]
#             dy = y_proj - self.target_pos[1]
            
#             total_cost += dx*dx + dy*dy

#         problem.minimize(total_cost + 1 * v0)
#         # problem.minimize(total_cost + 1.0 * T_flight)
        
#         solve_start = time.perf_counter()
#         status = problem.solve()
#         solve_end = time.perf_counter()
        
#         return {
#             "status": status,
#             "v": v0.value(),
#             "angle_deg": math.degrees(theta.value()),
#             "yaw_deg": math.degrees(phi.value()),
#             "T": T_flight.value(),
#             "cost": total_cost.value(),
#             "solve_time": solve_end - solve_start
#         }

# # # # Configuration
# # # MIN_DIST = 1.0
# # # MAX_DIST = 5.0
# # # NUM_POINTS = 100  # How many rows in the table
# # # SHOOTER_HEIGHT = 0.56 # Meters (approximate robot shooter height)

# # # # Use the target from the optimizer class logic
# # # TARGET_POS = np.array([4.625594, 4.034536, 1.83])

# # # def generate_table():
# # #     distances = np.linspace(MIN_DIST, MAX_DIST, NUM_POINTS)
# # #     results = []
    
# # #     print(f"\n{'Dist (m)':<10} | {'Vel (m/s)':<10} | {'Angle (deg)':<10} | {'Time (s)':<10} | {'Status'}")
# # #     print("-" * 75)

# # #     for dist in distances:
# # #         # Create a shooter position 'dist' away from target
# # #         # We assume the shooter is straight in line on X axis relative to target for generation
# # #         shooter_pos = np.array([
# # #             TARGET_POS[0] - dist, 
# # #             TARGET_POS[1], 
# # #             SHOOTER_HEIGHT
# # #         ])
        
# # #         # Static Optimization (Robot Velocity = 0)
# # #         shooter_vel = np.array([0.0, 0.0, 0.0])
        
# # #         optimizer = SleipnirRobustOptimizer(
# # #             shooter_pos=shooter_pos,
# # #             shooter_vel=shooter_vel,
# # #             min_v=5.0,
# # #             max_v=12.0,
# # #             min_angle_deg=60,
# # #             max_angle_deg=87.0
# # #         )
        
# # #         res = optimizer.optimize()
        
# # #         if "SUCCESS" in str(res['status']):
# # #             # Clean data
# # #             row = {
# # #                 "Distance": round(dist, 3),
# # #                 "Velocity": round(res['v'], 3),
# # #                 "Angle": round(res['angle_deg'], 3),
# # #                 "Time": round(res['T'], 4)
# # #             }
# # #             results.append(row)
# # #             print(f"{dist:<10.2f} | {res['v']:<10.4f} | {res['angle_deg']:<10.4f} | {res['T']:<10.4f} | OK")
# # #         else:
# # #             print(f"{dist:<10.2f} | {'---':<10} | {'---':<10} | {'---':<10} | FAIL")

# # #     # Export
# # #     df = pd.DataFrame(results)
# # #     csv_name = "shot_lookup_table.csv"
# # #     df.to_csv(csv_name, index=False)
# # #     print("\n" + "="*30)
# # #     print(f"saved to {csv_name}")
# # #     print("="*30)
    
# # # if __name__ == "__main__":
# # #     generate_table()

# def create_lookup_table():
#         min_dist = 0.7
#         max_dist = 8
#         num_points = 20
#         shooter_height = 0.52
#         hub_pos = np.array([4.625594, 4.034536, 1.83])
#         distances = np.linspace(min_dist, max_dist, num_points)
#         results = []
#         dist_lookup_table = LookupTableAll()
#         for dist in distances:
#             shooter_pos = np.array([
#                 hub_pos[0] - dist, 
#                 hub_pos[1], 
#                 shooter_height])
#             shooter_vel = np.array([0.0, 0.0, 0.0])
#             optimizer = SleipnirRobustOptimizer(
#             shooter_pos=shooter_pos,
#             shooter_vel=shooter_vel,
#             min_v=1.5,
#             max_v=9.2,
#             min_angle_deg=61,
#             max_angle_deg=87.0
#         )
        
#             res = optimizer.optimize()

#             if "SUCCESS" in str(res['status']):
#                 dist_lookup_table.add_entry(round(dist, 3), round(res['v'], 3), round(res['angle_deg']), round(res['T'], 4))
#                 print(f"self.dist_lookup_table.add_entry({round(dist, 3)}, {round(res['v'], 3)}, {round(res['angle_deg'], 3)}, {round(res['T'], 3)})")

# create_lookup_table()