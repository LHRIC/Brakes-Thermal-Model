import numpy as np
import matplotlib.pyplot as plt

def get_material_properties(mat_type, T_c):
    """
    Returns (rho_metal, k_metal, Cp_metal) for a given material type and temperature T_c (°C)
    ADD DEFINITIONS AS NEEDED.
    """
    if mat_type == 1:
        # AISI 410 stainless-steel
        rho_metal = 7740  # kg/m^3
        if T_c < 100.0:
            k_metal  = 24.9        # [W/m-K]
            Cp_metal = 460.0       # [J/kg-K]
        else:
            k_metal  = 0.0095 * T_c + 23.95
            Cp_metal = 0.1 * T_c + 450.0

    elif mat_type == 2:
        # AISI 4130 annealed steel
        rho_metal = 7850  # kg/m^3
        k_metal  = 36.32  # [W/m-K]
        Cp_metal = 600.0  # [J/kg-K]

    else:
        rho_metal = 7800.0
        k_metal   = 40.0
        Cp_metal  = 500.0

    return rho_metal, k_metal, Cp_metal

# Helper Functions for Heat Flows

def convection_coefficient(omega, h_base=25.0, omega_ref=100.0, exponent=0.7):
    """
    Convection coefficient as a function of wheel (rotor) speed.
    h(omega) = h_base * (omega / omega_ref)^exponent
    TO BE ADJUSTED
    """
    if omega_ref < 1e-6:
        omega_ref = 1.0
    return h_base * (max(omega, 0.01) / omega_ref)**exponent

def conduction_loss_to_hub(T_rotor, rotor_area, T_hub=30.0, k_cond=5.0):
    """
    Very simple conduction model:
    Q_dot_cond = k_cond * rotor_area * (T_rotor - T_hub)
    TO BE ADJUSTED?
    """
    return k_cond * rotor_area * (T_rotor - T_hub)

def compute_brake_energy(I_wheel, omega1, omega2):
    """
    Computes the change in rotational kinetic energy: 0.5 * I_wheel * (omega1^2 - omega2^2)
    """
    return 0.5 * I_wheel * (omega1**2 - omega2**2)

# Main Simulation Function
def simulate_multiple_rotors(rotor_pairs, events, ambient_temp, time_step):
    """
    Lumped thermal simulation with:
        - random braking events
        - convection as function of speed
        - conduction to hub
        - optional partition fraction to rotor
        - temperature-dependent material properties

    Parameters:
        rotor_pairs: list of pairs, each pair is [rotor_front_dict, rotor_rear_dict] (or any grouping)
                     each rotor dict has keys:
                        "mass_disc", "area_disc", "fraction_into_disc", "I_wheel",
                        "emissivity", "material_type"
                        (optional: "mass_disc" might be computed from geometry + rho, or just measured mass)
        events:      list of dicts, each with:
                        "omega_init", "omega_final", "t_brake", "t_cool"
        ambient_temp (°C)
        time_step (s)
    Returns:
        time_array (1D np.array)
        rotor_temp_history (2D np.array, shape=(num_rotors, len(time_array)))
        brake_flag (1D np.array) of 0 or 1, indicating cooling or braking
    """
    sigma_sb = 5.670374419e-8
    num_rotors = len(rotor_pairs) * 2

    # initialize rotor temperatures (°C)
    rotor_temp = np.array([ambient_temp]*num_rotors, dtype=float)

    time_history = []
    brake_flag_history = []
    rotor_temp_list = []

    current_time = 0.0

    def run_phase(n_steps, power_in_array, omega_array, phase_flag):
        """
        n_steps: how many sub-steps in the phase
        power_in_array: net friction heat input per rotor (W)
        omega_array: array of rotor speeds for each rotor (or a single speed if they're the same)
        phase_flag: 1 = braking, 0 = cooling
        """
        nonlocal current_time, rotor_temp

        for step_i in range(n_steps):
            new_temps = np.zeros_like(rotor_temp)
            for i in range(num_rotors):
                pair_idx, rotor_idx = divmod(i, 2)
                rotor = rotor_pairs[pair_idx][rotor_idx]

                T_c = rotor_temp[i]
                mat_type = rotor["material_type"]

                # retrieve temp dependent properties
                rho_metal, k_metal, Cp_metal = get_material_properties(mat_type, T_c)

                mass_disc = rotor["mass_disc"]

                heat_capacity = mass_disc * Cp_metal

                # convection
                h_now = convection_coefficient(omega_array[i])
                Q_conv = h_now * rotor["area_disc"] * ((T_c + 273.15) - (ambient_temp + 273.15))

                # radiation
                eps = rotor["emissivity"]
                T_k  = T_c + 273.15
                Ta_k = ambient_temp + 273.15
                Q_rad = eps * sigma_sb * rotor["area_disc"] * (T_k**4 - Ta_k**4)

                # conduction to hub (assumed TO BE ADJUSTED?)
                Q_cond = conduction_loss_to_hub(T_c, rotor["area_disc"])

                # net heat
                Q_in = power_in_array[i]
                Q_net = Q_in - Q_conv - Q_rad - Q_cond

                # temperature increment
                dT = (Q_net / heat_capacity) * time_step
                new_temps[i] = T_c + dT

            rotor_temp = new_temps
            current_time += time_step

            time_history.append(current_time)
            brake_flag_history.append(phase_flag)
            rotor_temp_list.append(rotor_temp.copy())

    # loop over the brake events
    for ev in events:
        omega_i = ev["omega_init"]
        omega_f = ev["omega_final"]
        t_brake = ev["t_brake"]
        t_cool  = ev["t_cool"]

        # 1) Compute total brake energy from one wheel's inertia or handle differently if needed
        #    If each rotor has its own I_wheel, you might do that individually.
        #    We'll do a simpler approach: total rotor-based energy in a loop, or just single-lump for demonstration.
        #    For example, let's sum up the rotor inertias, or handle them rotor-by-rotor below.

        brake_power = np.zeros(num_rotors, dtype=float)

        for i in range(num_rotors):
            pair_idx, rotor_idx = divmod(i, 2)
            rotor = rotor_pairs[pair_idx][rotor_idx]
            # rotational KE difference
            dE = compute_brake_energy(rotor["I_wheel"], omega_i, omega_f)
            # fraction going into the disc
            fraction_disc = rotor["fraction_into_disc"]
            # power spread out over t_brake
            if t_brake > 0.0:
                brake_power[i] = (dE * fraction_disc) / t_brake

        # braking phase
        if t_brake > 0.0:
            n_steps_brake = int(t_brake / time_step)
            if n_steps_brake < 1:
                n_steps_brake = 1

            # for more accurate approach, linearly reduce speed from omega_i to omega_f
            speed_vals = np.linspace(omega_i, omega_f, n_steps_brake)
            for step_i in range(n_steps_brake):
                omega_array = np.array([speed_vals[step_i]] * num_rotors)
                run_phase(1, brake_power, omega_array, phase_flag=1)
        else:
            # no actual brake phase
            pass

        # cooling phase
        if t_cool > 0.0:
            n_steps_cool = int(t_cool / time_step)
            if n_steps_cool < 1:
                n_steps_cool = 1

            # wheel speed is just final, or near 0
            omega_cool = omega_f
            zero_power = np.zeros(num_rotors)
            for step_i in range(n_steps_cool):
                omega_array = np.array([omega_cool]*num_rotors)
                run_phase(1, zero_power, omega_array, phase_flag=0)

    time_array = np.array(time_history)
    rotor_temp_history = np.array(rotor_temp_list).T  # shape => (num_rotors, timesteps)
    brake_flag = np.array(brake_flag_history)

    return time_array, rotor_temp_history, brake_flag

if __name__ == "__main__":
    rotor_pairs = [
        [
            {
                "mass_disc": 0.5289,
                "area_disc": 0.055,
                "fraction_into_disc": 0.9,
                "I_wheel": 5.0213,
                "emissivity": 0.6,
                "material_type": 2  # AISI 4130
            },
            {
                "mass_disc": 0.3778,
                "area_disc": 0.0347,
                "fraction_into_disc": 0.9,
                "I_wheel": 3.542,
                "emissivity": 0.6,
                "material_type": 2  # AISI 4130
            }
        ],
        [
            {
                "mass_disc": 0.500,
                "area_disc": 0.050,
                "fraction_into_disc": 0.9,
                "I_wheel": 4.8,
                "emissivity": 0.75,
                "material_type": 1  #AISI 410
            },
            {
                "mass_disc": 0.360,
                "area_disc": 0.033,
                "fraction_into_disc": 0.9,
                "I_wheel": 3.4,
                "emissivity": 0.75,
                "material_type": 1  # AISI 410
            }
        ]
    ]

    # generate random events: speed range (60–100 rad/s)
    # eventually replace with wheel speed sensor
    np.random.seed(42)
    events = []
    for _ in range(100):
        omega_i = np.random.uniform(60, 100)
        reduce_speed = np.random.uniform(0, 30)
        omega_f = max(0, omega_i - reduce_speed)
        t_brake = np.random.uniform(1, 3)
        t_cool  = np.random.uniform(2, 5)
        events.append({
            "omega_init": omega_i,
            "omega_final": omega_f,
            "t_brake": t_brake,
            "t_cool": t_cool
        })

    # simulate
    ambient_temp = 25.0
    time_step = 0.1
    time_array, rotor_temps, brake_flag = simulate_multiple_rotors(
        rotor_pairs, events, ambient_temp, time_step
    )

    # analyze peak temperatures
    for i in range(len(rotor_temps)):
        peak_temp = np.max(rotor_temps[i, :])
        peak_t_idx = np.argmax(rotor_temps[i, :])
        peak_time = time_array[peak_t_idx]
        print(f"[Rotor {i+1}] Peak Temp: {peak_temp:.1f} °C ("
              f"{peak_temp*9/5+32:.1f} °F) at t={peak_time:.1f}s")

    material_names = {
        1: "AISI 410",
        2: "AISI 4130",
        # Add more if needed
    }

    # Plot
    fig, axarr = plt.subplots(3, 1, gridspec_kw={'height_ratios': [3, 3, 1]}, sharex=True)

    num_rotors = len(rotor_temps)

    for i in range(num_rotors):
        # identify which rotor dictionary this i corresponds to
        pair_idx, rotor_idx = divmod(i, 2)
        rotor_info = rotor_pairs[pair_idx][rotor_idx]

        mat_type = rotor_info["material_type"]
        mat_name = material_names.get(mat_type, f"Material {mat_type}")

        label_str = f"{mat_name}"

        # front rotors on axarr[0] and rears on axarr[1]
        if i % 2 == 0:  # front rotor 
            axarr[0].plot(time_array, rotor_temps[i, :], label=label_str)
        else:           # rear rotor
            axarr[1].plot(time_array, rotor_temps[i, :], label=label_str)

    # label subplots
    axarr[0].set_ylabel("Temperature (°C)")
    axarr[0].legend()
    axarr[0].grid(True)
    axarr[0].set_title("Front Rotor Temperatures Over Time")

    axarr[1].set_ylabel("Temperature (°C)")
    axarr[1].legend()
    axarr[1].grid(True)
    axarr[1].set_title("Rear Rotor Temperatures Over Time")

    axarr[2].step(time_array, brake_flag, where="post", color='red', alpha=0.7, label="Braking Events")
    axarr[2].set_ylabel("Brake Flag")
    axarr[2].set_yticks([0, 1])
    axarr[2].set_yticklabels(["Cooling", "Braking"])
    axarr[2].set_xlabel("Time (s)")
    axarr[2].legend()
    axarr[2].grid(True)

    plt.tight_layout()
    plt.show()
