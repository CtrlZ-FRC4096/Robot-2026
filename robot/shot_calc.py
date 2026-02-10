import numpy as np
import math
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from sleipnir.optimization import Problem
import sleipnir.autodiff as ad

class SleipnirRobustOptimizer:
    def __init__(self, shooter_pos=np.array([0.0, 0.0, 0.0]), shooter_vel=np.array([0.0, 0.0, 0.0]), min_v=0.0, max_v=25.0, min_angle_deg=0.0, max_angle_deg=85.0):
        
        # Constants
        self.gravity = 9.81
        self.air_density = 1.204
        self.mass = 0.216
        self.diameter = 0.15
        self.area = np.pi * (self.diameter / 2)**2
        self.drag_coefficient = 0.5
        
        self.magnus_enabled = True
        self.spin = -0.3
        self.lift_factor = 0.2
        
        # Target
        self.target_pos = np.array([4.625594, 4.034536, 1.83])
        self.shooter_pos = shooter_pos
        self.shooter_vel = shooter_vel
        
        # Constraints
        self.max_v = max_v
        self.min_v = min_v
        self.min_angle_rad = math.radians(min_angle_deg)
        self.max_angle_rad = math.radians(max_angle_deg)
        
        # Uncertainty
        self.v_std = 0.25
        self.theta_std = math.radians(1.0)
        self.phi_std = math.radians(2.0)

        # Opt config
        self.steps = 2
        
    def dynamics(self, state, spin_axis=None):
        # State: [x, y, z, vx, vy, vz]
        vx = state[3]
        vy = state[4]
        vz = state[5]
        
        # Velocity magnitude
        # Using ad.sqrt
        v_sq = vx*vx + vy*vy + vz*vz
        v_mag = ad.sqrt(v_sq)
        
        # Drag
        k_drag = -0.5 * self.air_density * self.drag_coefficient * self.area * v_mag
        
        ax_drag = k_drag * vx / self.mass
        ay_drag = k_drag * vy / self.mass
        az_drag = k_drag * vz / self.mass
        
        # Magnus
        if spin_axis is None:
            axis_x = 0.0
            axis_y = -1.0
            axis_z = 0.0
        else:
            axis_x = spin_axis[0]
            axis_y = spin_axis[1]
            axis_z = spin_axis[2]
        
        # Cross product (axis x v)
        cp_x = axis_y * vz - axis_z * vy 
        cp_y = axis_z * vx - axis_x * vz 
        cp_z = axis_x * vy - axis_y * vx 
        
        cp_mag = ad.sqrt(cp_x*cp_x + cp_y*cp_y + cp_z*cp_z)
        
        # Lift mag
        cl = self.lift_factor * abs(self.spin)
        
        # Force = 0.5 * rho * v^2 * A * Cl
        # Dir = cp / cp_mag
        c_lift_mass = (0.5 * self.air_density * self.area * cl) / self.mass
        
        denom = cp_mag + 1e-9
        
        ax_magnus = c_lift_mass * v_sq * (cp_x / denom)
        ay_magnus = c_lift_mass * v_sq * (cp_y / denom)
        az_magnus = c_lift_mass * v_sq * (cp_z / denom)
        
        # Gravity
        az_grav = -self.gravity
        
        return [
            vx, vy, vz,
            ax_drag + ax_magnus,
            ay_drag + ay_magnus,
            az_drag + az_magnus + az_grav
        ]

    def run_rk2(self, state, dt, spin_axis=None):
        # Heuns method (RK2) - 2 evaluations per step vs 4 for RK4
        k1 = self.dynamics(state, spin_axis)
        
        state_k2 = [s + k * dt for s, k in zip(state, k1)]
        k2 = self.dynamics(state_k2, spin_axis)
        
        new_state = []
        for i in range(6):
            val = state[i] + (dt / 2.0) * (k1[i] + k2[i])
            new_state.append(val)
            
        return new_state

    def run_rk4(self, state, dt, spin_axis=None):
        k1 = self.dynamics(state, spin_axis)
        
        state_k2 = [s + k * dt / 2 for s, k in zip(state, k1)]
        k2 = self.dynamics(state_k2, spin_axis)
        
        state_k3 = [s + k * dt / 2 for s, k in zip(state, k2)]
        k3 = self.dynamics(state_k3, spin_axis)
        
        state_k4 = [s + k * dt for s, k in zip(state, k3)]
        k4 = self.dynamics(state_k4, spin_axis)
        
        new_state = []
        for i in range(6):
            val = state[i] + (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
            new_state.append(val)
            
        return new_state

    def optimize(self):
        # Strategy 1: Fast Sweep (Steps=2, Default Guess)
        self.steps = 2
        res = self._solve_problem(guess_type="default")

        status_str = str(res['status'])
        # If successful and accurate, return
        if "SUCCESS" in status_str and res['cost'] <= 10.0:
            return res

        # Strategy 2: High Precision (Steps=20, High Lob Guess)
        # We jump straight to High Lob guess because if default failed at steps=2,
        # it likely needs a higher angle to clear the "drag wall" or avoid local minima.
        res = self._solve_problem(guess_type="high_lob")
        
        status_str = str(res['status'])
        if "SUCCESS" in status_str and res['cost'] <= 10.0:
             return res
        
        # If needed we can increase the solver steps but it seems fine at 2
        # self.steps = 20
        # Strategy 3: High Precision (Steps=20, Max Energy/Lower Angle Guess)
        res = self._solve_problem(guess_type="min_angle")
            
        return res

    def _solve_problem(self, guess_type="default", use_rk2=False, reduce_perturbations=False):
        problem = Problem()
        
        # Smart Initialization
        dx = self.target_pos[0] - self.shooter_pos[0]
        dy = self.target_pos[1] - self.shooter_pos[1]
        dist = math.sqrt(dx*dx + dy*dy)
        yaw_init = math.atan2(dy, dx)
        
        # Decision Variables
        v0 = problem.decision_variable()
        theta = problem.decision_variable()
        
        # -- Initial Guess Logic --
        if guess_type == "high_lob":
            # Guess high angle (e.g. 75 deg), moderate velocity
            th_guess = min(math.radians(75.0), self.max_angle_rad)
            v_guess = 10.0
        elif guess_type == "min_angle":
            # Guess minimum angle (more direct shot), max velocity
            th_guess = self.min_angle_rad + 0.1 # slightly above min
            v_guess = self.max_v
        else: # default
            # Bias towards 60 degrees for lobs
            mid_angle = (self.min_angle_rad + self.max_angle_rad) / 2.0
            lob_bias = math.radians(60.0) 
            th_guess = max(self.min_angle_rad, min(lob_bias, self.max_angle_rad))
            v_guess = max(self.min_v, min(10.0, self.max_v))

        v0.set_value(v_guess)
        theta.set_value(th_guess)
        # -------------------------
        
        phi = problem.decision_variable()
        phi.set_value(yaw_init)
        
        T_flight = problem.decision_variable()
        # Estimate T based on guess
        v_xy = v_guess * math.cos(th_guess)
        t_guess = dist / (v_xy + 1e-9)
        T_flight.set_value(t_guess)
        
        # Bounds
        problem.subject_to(v0 >= self.min_v)
        problem.subject_to(v0 <= self.max_v)
        
        problem.subject_to(theta >= self.min_angle_rad)
        problem.subject_to(theta <= self.max_angle_rad)

        # Yaw bounds (allow +/- 45 deg from direct line)
        base_yaw = math.atan2(self.target_pos[1] - self.shooter_pos[1], self.target_pos[0] - self.shooter_pos[0])
        problem.subject_to(phi >= base_yaw - 0.8)
        problem.subject_to(phi <= base_yaw + 0.8)
        
        problem.subject_to(T_flight >= 0.1)
        problem.subject_to(T_flight <= 3.0)
        
        dt = T_flight / self.steps
        
        if reduce_perturbations:
            # Fast Mode: Check primary axes only (positive direction)
            # 4 Total Scenarios
            perturbations = [
                (0.0, 0.0, 0.0),
                (self.v_std, 0.0, 0.0),
                (0.0, self.theta_std, 0.0),
                (0.0, 0.0, self.phi_std),
            ]
        else:
            # Full Robust Mode: Check "Corners" of the uncertainty volume
            # This captures compound errors (e.g. High Velocity + Low Angle + Left Drift)
            # 1 Nominal + 8 Corners = 9 Total Scenarios
            perturbations = [(0.0, 0.0, 0.0)]
            for s_v in [self.v_std, -self.v_std]:
                for s_th in [self.theta_std, -self.theta_std]:
                    for s_ph in [self.phi_std, -self.phi_std]:
                        perturbations.append((s_v, s_th, s_ph))
        
        total_cost = ad.Variable(0.0)
        
        for dv, dtheta, dphi in perturbations:
            v_curr = v0 + dv
            theta_curr = theta + dtheta
            phi_curr = phi + dphi
            
            vx = v_curr * ad.cos(theta_curr) * ad.cos(phi_curr) + self.shooter_vel[0]
            vy = v_curr * ad.cos(theta_curr) * ad.sin(phi_curr) + self.shooter_vel[1]
            vz = v_curr * ad.sin(theta_curr) + self.shooter_vel[2]
            
            # Spin axis depends on Yaw (phi)
            spin_axis = [ad.sin(phi_curr), -ad.cos(phi_curr), 0.0]

            # Initial state
            state = [
                ad.Variable(self.shooter_pos[0]), 
                ad.Variable(self.shooter_pos[1]), 
                ad.Variable(self.shooter_pos[2]),
                vx, vy, vz
            ]
            
            # Simulate
            for _ in range(self.steps):
                if use_rk2:
                    state = self.run_rk2(state, dt, spin_axis)
                else:
                    state = self.run_rk4(state, dt, spin_axis)
                
            x_final = state[0]
            y_final = state[1]
            z_final = state[2]
            vx_final = state[3]
            vy_final = state[4]
            vz_final = state[5]

            if dv == 0 and dtheta == 0 and dphi == 0:
                problem.subject_to(z_final == self.target_pos[2])
                # Uncomment this if you want to fall into the hoop at -2m/s
                problem.subject_to(vz_final < -1) 
                
            # Cost calc (Minimize X-Y error at Target Z Height)
            # dt_corr: time diff to reach exact Z plane
            dt_corr = (self.target_pos[2] - z_final) / (vz_final + 1e-9)
            
            x_proj = x_final + vx_final * dt_corr
            y_proj = y_final + vy_final * dt_corr
            
            dx = x_proj - self.target_pos[0]
            dy = y_proj - self.target_pos[1]
            
            total_cost += dx*dx + dy*dy

        problem.minimize(total_cost + 1 * v0)
        # problem.minimize(total_cost + 1.0 * T_flight)
        
        solve_start = time.perf_counter()
        status = problem.solve()
        solve_end = time.perf_counter()
        
        return {
            "status": status,
            "v": v0.value(),
            "angle_deg": math.degrees(theta.value()),
            "yaw_deg": math.degrees(phi.value()),
            "T": T_flight.value(),
            "cost": total_cost.value(),
            "solve_time": solve_end - solve_start
        }

if __name__ == "__main__":

    initial_vel = np.array([1.0, 2.0, 0.0]) 
    initial_pos = np.array([4.0, 0.0, 0.0]) 
    
    solver = SleipnirRobustOptimizer(
        shooter_pos=initial_pos,
        shooter_vel=initial_vel, 
        min_v=6.0,
        max_v=12.0, 
        min_angle_deg=60,
        max_angle_deg=87,
    )
    
    print(f"Running Sleipnir Optimizer (Initial Shooter Vel: {initial_vel})...")
    
    start_time = time.perf_counter()
    res = solver.optimize()
    end_time = time.perf_counter()

    print("-" * 30)
    print("SLEIPNIR OPTIMIZATION RESULT")
    print("-" * 30)
    print(f"Total Runtime: {end_time - start_time:.4f} s")
    print(f"Solver Time:   {res['solve_time']:.4f} s")
    print(f"Python Setup:  {(end_time - start_time) - res['solve_time']:.4f} s")
    print(f"Status: {res['status']}")
    print(f"Optimal V:     {res['v']:.4f} m/s")
    print(f"Optimal Theta: {res['angle_deg']:.4f} degrees")
    print(f"Optimal Yaw:   {res['yaw_deg']:.4f} degrees")
    print(f"Flight Time:   {res['T']:.4f} s")
    print(f"Cost (SSR):    {res['cost']:.6f}")

    # --- Full Uncertainty Report ---
    print("\n" + "=" * 80)
    print("FULL UNCERTAINTY REPORT")
    print("=" * 80)
    
    def simulate_shot_report(v_shot, theta_shot_deg, yaw_shot_deg=None, verbose=False):
        theta_rad = math.radians(theta_shot_deg)
        if yaw_shot_deg is None:
            yaw_shot_deg = res['yaw_deg']
        phi_rad = math.radians(yaw_shot_deg)

        dt = res['T'] / solver.steps
        
        vx = v_shot * math.cos(theta_rad) * math.cos(phi_rad) + solver.shooter_vel[0]
        vy = v_shot * math.cos(theta_rad) * math.sin(phi_rad) + solver.shooter_vel[1]
        vz = v_shot * math.sin(theta_rad) + solver.shooter_vel[2]
        
        spin_axis = [math.sin(phi_rad), -math.cos(phi_rad), 0.0]
        
        state = [
            solver.shooter_pos[0], solver.shooter_pos[1], solver.shooter_pos[2],
            vx, vy, vz
        ]
        
        def step(s, dt_step):
            x, y, z, _vx, _vy, _vz = s
            v_sq = _vx*_vx + _vy*_vy + _vz*_vz
            v_mag = math.sqrt(v_sq)
            
            # Drag
            k_drag = -0.5 * solver.air_density * solver.drag_coefficient * solver.area * v_mag
            ax_drag = k_drag * _vx / solver.mass
            ay_drag = k_drag * _vy / solver.mass
            az_drag = k_drag * _vz / solver.mass
            
            # Magnus (using computed axis)
            axis_x, axis_y, axis_z = spin_axis
            
            cp_x = axis_y * _vz - axis_z * _vy
            cp_y = axis_z * _vx - axis_x * _vz
            cp_z = axis_x * _vy - axis_y * _vx
            cp_mag = math.sqrt(cp_x*cp_x + cp_y*cp_y + cp_z*cp_z)
            
            cl = solver.lift_factor * abs(solver.spin)
            c_lift_mass = (0.5 * solver.air_density * solver.area * cl) / solver.mass
            
            denom = cp_mag + 1e-9
            ax_magnus = c_lift_mass * v_sq * (cp_x / denom)
            ay_magnus = c_lift_mass * v_sq * (cp_y / denom)
            az_magnus = c_lift_mass * v_sq * (cp_z / denom)
            
            az_grav = -solver.gravity
            
            return [
                _vx, _vy, _vz, 
                ax_drag + ax_magnus, 
                ay_drag + ay_magnus, 
                az_drag + az_magnus + az_grav
            ]
            
        def rk4_step(s, _dt):
            k1 = step(s, _dt)
            sk2 = [v + k * _dt/2 for v, k in zip(s, k1)]
            k2 = step(sk2, _dt)
            sk3 = [v + k * _dt/2 for v, k in zip(s, k2)]
            k3 = step(sk3, _dt)
            sk4 = [v + k * _dt for v, k in zip(s, k3)]
            k4 = step(sk4, _dt)
            return [v + (_dt/6)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) for i, v in enumerate(s)]

        # Run integration
        traj = [state]
        for _ in range(solver.steps):
            state = rk4_step(state, dt)
            traj.append(state)
            
        final_state = state
        
        # Calculate impact at target Z plane (Horizontal Goal)
        x_f, y_f, z_f, vx_f, vy_f, vz_f = final_state
        
        # Time correction to hit plane Z = target_z
        dt_corr = (solver.target_pos[2] - z_f) / (vz_f + 1e-9)
        
        # Projected impact
        impact_x = x_f + vx_f * dt_corr
        impact_y = y_f + vy_f * dt_corr
        
        dist_error = math.sqrt((impact_x - solver.target_pos[0])**2 + (impact_y - solver.target_pos[1])**2)
        radius = 0.5
        hit = dist_error <= radius
        
        return {
            "v_shot": v_shot,
            "theta_shot": theta_shot_deg,
            "impact_x": impact_x,
            "dist_error": dist_error,
            "hit": hit,
            "vz_final": vz_f
        }

    # Nominal and Perturbed Cases
    cases = [
        ("Nominal", 0, 0),
        ("+1 Sigma V", solver.v_std, 0),
        ("-1 Sigma V", -solver.v_std, 0),
        ("+1 Sigma Angle", 0, math.degrees(solver.theta_std)),
        ("-1 Sigma Angle", 0, -math.degrees(solver.theta_std)),
        ("+2 Sigma V", 2*solver.v_std, 0),
        ("-2 Sigma V", -2*solver.v_std, 0),
        ("+2 Sigma Angle", 0, 2*math.degrees(solver.theta_std)),
        ("-2 Sigma Angle", 0, -2*math.degrees(solver.theta_std)),
    ]
    
    print(f"{'Condition':<18} | {'V (m/s)':<8} | {'Theta':<6} | {'Impact X':<8} | {'Dist Err':<10} | {'Vz Final':<8} | {'Hit?'}")
    print("-" * 95)
    
    opt_v = res['v']
    opt_theta = res['angle_deg']
    
    for label, dv, dtheta in cases:
        r = simulate_shot_report(opt_v + dv, opt_theta + dtheta)
        hit_str = "YES" if r['hit'] else "NO"
        print(f"{label:<18} | {r['v_shot']:<8.4f} | {r['theta_shot']:<6.2f} | {r['impact_x']:<8.4f} | {r['dist_error']:<10.4f} | {r['vz_final']:<8.4f} | {hit_str}")
    
    print("-" * 95)

    # --- Monte Carlo Simulation ---
    print("\nRunning Monte Carlo Accuracy Simulation (N=1000)...")
    mc_start = time.perf_counter()
    mc_hits = 0
    mc_total = 1000
    np.random.seed(4096) # Fixed seed for consistency
    
    mc_v = np.random.normal(res['v'], solver.v_std, mc_total)
    mc_theta = np.random.normal(res['angle_deg'], math.degrees(solver.theta_std), mc_total)
    mc_yaw = np.random.normal(res['yaw_deg'], math.degrees(solver.phi_std), mc_total)
    
    for v_s, th_s, yaw_s in zip(mc_v, mc_theta, mc_yaw):
        if simulate_shot_report(v_s, th_s, yaw_s)['hit']:
            mc_hits += 1
            
    mc_accuracy = (mc_hits / mc_total) * 100.0
    print(f"Monte Carlo Accuracy: {mc_accuracy:.1f}% (Time: {time.perf_counter() - mc_start:.3f}s)")

    # --- Visualization ---
    try:
        # Switch to 3D Plotting
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Physics helper for plotting (fully 3D)
        def step_3d(s, dt_step, spin_axis):
            x, y, z, vx, vy, vz = s
            v_sq = vx*vx + vy*vy + vz*vz
            v_mag = math.sqrt(v_sq)
            
            # Drag
            k_drag = -0.5 * solver.air_density * solver.drag_coefficient * solver.area * v_mag
            ax_d = k_drag * vx / solver.mass
            ay_d = k_drag * vy / solver.mass
            az_d = k_drag * vz / solver.mass
            
            # Magnus
            axis_x, axis_y, axis_z = spin_axis
            cp_x = axis_y * vz - axis_z * vy
            cp_y = axis_z * vx - axis_x * vz
            cp_z = axis_x * vy - axis_y * vx
            cp_mag = math.sqrt(cp_x*cp_x + cp_y*cp_y + cp_z*cp_z)
            
            if cp_mag > 1e-9:
                cl = solver.lift_factor * abs(solver.spin)
                f_lift = 0.5 * solver.air_density * v_sq * solver.area * cl
                # Dir = cp / cp_mag
                c_lift_mass_eff = f_lift / (solver.mass * cp_mag)
                
                ax_m = c_lift_mass_eff * cp_x
                ay_m = c_lift_mass_eff * cp_y
                az_m = c_lift_mass_eff * cp_z
            else:
                ax_m, ay_m, az_m = 0, 0, 0

            return [vx, vy, vz, ax_d + ax_m, ay_d + ay_m, az_d + az_m - solver.gravity]

        def rk4_3d(s, dt_val, spin_axis):
            k1 = step_3d(s, dt_val, spin_axis)
            sk2 = [v + k * dt_val/2 for v, k in zip(s, k1)]
            k2 = step_3d(sk2, dt_val, spin_axis)
            sk3 = [v + k * dt_val/2 for v, k in zip(s, k2)]
            k3 = step_3d(sk3, dt_val, spin_axis)
            sk4 = [v + k * dt_val for v, k in zip(s, k3)]
            k4 = step_3d(sk4, dt_val, spin_axis)
            return [v + (dt_val/6)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) for i, v in enumerate(s)]
            
        def get_trajectory_3d(v_shot, theta_deg, yaw_deg, T_flight, steps=50):
            dt_plot = T_flight / steps
            theta_rad = math.radians(theta_deg)
            phi_rad = math.radians(yaw_deg)
            
            # Axis (spin perpendicular to velocity in XY plane for backspin)
            spin_axis = [math.sin(phi_rad), -math.cos(phi_rad), 0.0]
            
            vx = v_shot * math.cos(theta_rad) * math.cos(phi_rad) + solver.shooter_vel[0]
            vy = v_shot * math.cos(theta_rad) * math.sin(phi_rad) + solver.shooter_vel[1]
            vz = v_shot * math.sin(theta_rad) + solver.shooter_vel[2]
            
            state = [solver.shooter_pos[0], solver.shooter_pos[1], solver.shooter_pos[2], vx, vy, vz]
            xs, ys, zs = [state[0]], [state[1]], [state[2]]
            
            for _ in range(steps):
                state = rk4_3d(state, dt_plot, spin_axis)
                xs.append(state[0])
                ys.append(state[1])
                zs.append(state[2])
            return xs, ys, zs

        print("Generating 3D uncertainty cloud...")
        np.random.seed(42)
        v_samples = np.random.normal(res['v'], solver.v_std, 30) # Reduced count for performance
        theta_samples = np.random.normal(res['angle_deg'], math.degrees(solver.theta_std), 30)
        yaw_samples = np.random.normal(res['yaw_deg'], math.degrees(solver.phi_std), 30)
        
        for v_s, th_s, yaw_s in zip(v_samples, theta_samples, yaw_samples):
            xs_s, ys_s, zs_s = get_trajectory_3d(v_s, th_s, yaw_s, res['T'], steps=40)
            ax.plot(xs_s, ys_s, zs_s, color='gray', alpha=0.15, linewidth=1)

        xs, ys, zs = get_trajectory_3d(res['v'], res['angle_deg'], res['yaw_deg'], res['T'], steps=60)
        ax.plot(xs, ys, zs, 'b-', linewidth=2.5, label=f"Optimal Shot")
        
        t_pos = solver.target_pos
        theta_ring = np.linspace(0, 2*np.pi, 50)
        ring_r = 0.5
        ring_x = t_pos[0] + ring_r * np.cos(theta_ring)
        ring_y = t_pos[1] + ring_r * np.sin(theta_ring)
        ring_z = np.full_like(ring_x, t_pos[2])
        ax.plot(ring_x, ring_y, ring_z, 'r-', linewidth=3, label='Target Ring')
        
        ax.scatter(solver.shooter_pos[0], solver.shooter_pos[1], solver.shooter_pos[2], c='g', s=100, label='Shooter')
        
        v_robot = solver.shooter_vel
        v_robot_mag = np.linalg.norm(v_robot)
        if v_robot_mag > 0.1:
            ax.quiver(
                solver.shooter_pos[0], solver.shooter_pos[1], solver.shooter_pos[2],
                v_robot[0], v_robot[1], v_robot[2],
                length=1.0, normalize=False, color='orange', linewidth=2, arrow_length_ratio=0.3,
                label=f'Robot Vel ({v_robot_mag:.1f} m/s)'
            )
            
        yaw_rad_vis = math.radians(res['yaw_deg'])
        yaw_vec_x = math.cos(yaw_rad_vis)
        yaw_vec_y = math.sin(yaw_rad_vis)
        
        ax.quiver(
            solver.shooter_pos[0], solver.shooter_pos[1], solver.shooter_pos[2],
            yaw_vec_x, yaw_vec_y, 0,
            length=1.5, normalize=True, color='cyan', linewidth=2, arrow_length_ratio=0.3,
            label=f"Aim Yaw ({res['yaw_deg']:.1f}°)"
        )

        ax.plot([0, t_pos[0]], [0, t_pos[1]], [0, 0], 'k--', alpha=0.3)
        
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title(f"3D Opt: V={res['v']:.2f}, θ={res['angle_deg']:.1f}°, Yaw={res['yaw_deg']:.1f}°\nAccuracy: {mc_accuracy:.1f}%")
        
        ax.set_box_aspect([1,1,1])
        
        mid_x = (max(xs) + min(xs)) * 0.5
        mid_y = (max(ys) + min(ys)) * 0.5
        max_range = max(max(xs)-min(xs), max(ys)-min(ys), max(zs)) / 2.0
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(0, max(zs) + 1.0)
        
        ax.legend()

        plt.savefig("trajectory_3d.png")
        print("\n3D Plot saved to 'trajectory_3d.png'. Calculating display...")
        plt.show()

    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"\nCould not generate plot: {e}")
