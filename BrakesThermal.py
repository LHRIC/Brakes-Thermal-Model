import numpy as np
import matplotlib.pyplot as plt

def brake_energy_single_maneuver(m_car, I_wheels, v1, v2, drag=0.0, distance=0.0):
    """
    Compute the total energy (Joules) dissipated in a single braking event 
    from speed v1 to v2 (m/s). This includes:
      1. Translational KE: 0.5*m_car*(v1^2 - v2^2)
      2. Rotational KE: 0.5*I_wheels*(omega1^2 - omega2^2)
      3. (Optional) drag * distance (if you want to subtract aerodynamic drag portion)
    """
    # Translational kinetic energy
    dE_trans = 0.5 * m_car * (v1**2 - v2**2)
    
    # Rotational kinetic energy, assuming wheel speed = vehicle speed / R (but 
    # if you have an effective rolling radius, you can incorporate that into I_wheels):
    #   omega = v / r, so (omega1^2 - omega2^2) = (v1^2 - v2^2)/r^2
    #   => 0.5 * I_wheels * (v1^2 - v2^2)/r^2
    # For simplicity, we assume I_wheels is "adjusted" for the direct velocity reference:
    dE_rot = 0.5 * I_wheels * (v1**2 - v2**2)
    
    # Subtract aerodynamic drag (if you have that info)
    dE_drag = drag * distance  # simplistic placeholder
    
    return dE_trans + dE_rot - dE_drag

def lumped_brake_temp_sim(
    n_brakings=100,
    # Car / brake system properties:
    m_car=200.0,                 # kg (Formula Student style car)
    I_wheels=4.0,                # kg·m^2 (rotational inertia total)
    p=0.98,                      # fraction of heat going into rotor
    # Disc properties:
    m_disc=1.0,                  # kg mass of single disc
    cp_disc=460.0,               # J/(kg*K), steel
    A_disc=0.05,                 # m^2 effective area for conv/rad
    eps=0.8,                     # emissivity for radiation
    # Scenario settings:
    v1_kmh=70.0,                 # initial speed for each brake event (km/h)
    v2_kmh=45.0,                 # final speed for each brake event (km/h)
    brake_time=2.0,              # time spent braking for each event (s)
    cool_time=3.0,               # coasting/cooling time between events (s)
    T_init=80.0,                 # initial rotor temp (°C)
    T_amb=25.0,                  # ambient (°C)
    h_conv=30.0,                 # W/(m^2*K) - approximate conv. coeff
    include_radiation=False,
    dt=0.1                       # simulation time step
):
    """
    Simulate the rotor temperature evolution through multiple repeated braking events.
    Each event goes from v1 to v2 in 'brake_time', then there's a 'cool_time' 
    before the next braking. We assume repeated cycles of [brake -> cool -> brake -> cool ...].
    """
    # Convert speeds from km/h to m/s
    v1 = v1_kmh / 3.6
    v2 = v2_kmh / 3.6

    # Pre-calculate the total energy dissipated in ONE maneuver (Joules)
    dE_one = brake_energy_single_maneuver(m_car, I_wheels, v1, v2)
    # portion to disc
    dE_disc = p * dE_one

    # We'll do an explicit Euler approach, stepping through each brake/cool cycle
    # total time for 1 cycle:
    #   brake_time + cool_time
    n_cycles = n_brakings
    cycle_time = brake_time + cool_time
    total_time = n_cycles * cycle_time

    # Build a time array with small dt
    t_array = np.arange(0, total_time, dt)
    T_array = np.zeros_like(t_array)
    T_array[0] = T_init  # start rotor temperature

    # We'll keep track of whether we are "braking" or "cooling" at each moment
    # to apply the correct power input
    idx_max = len(t_array) - 1

    # Heat capacity (J/K) for the disc
    C_total = m_disc * cp_disc

    for i in range(idx_max):
        T_current = T_array[i]
        # figure out which cycle we are in
        cycle_index = int(np.floor(t_array[i] / cycle_time))  # 0..n_brakings-1
        time_in_cycle = t_array[i] - cycle_index*cycle_time

        # Decide if we are in braking phase or cooling
        if time_in_cycle < brake_time and cycle_index < n_brakings:
            # We are in "braking" portion
            # approximate constant power input over the brake_time
            # Power_in = dE_disc / brake_time
            Q_in = (dE_disc / brake_time)
        else:
            Q_in = 0.0  # no friction heating

        # Cooling (convection + optional radiation)
        # Q_conv = h*A*(Trotor - Tamb)
        Q_conv = h_conv * A_disc * ((T_current+273.15) - (T_amb+273.15))  # in Kelvin for clarity if you prefer
        if Q_conv < 0:
            Q_conv = 0  # if rotor < ambient, reverse sign is possible; for simple model, clamp or let it go negative

        # Radiation
        Q_rad = 0.0
        if include_radiation:
            # use Stefan-Boltzmann
            # Q_rad = eps * sigma * A_disc * (T^4 - T_amb^4) (in Kelvin)
            sigma = 5.670374419e-8
            T_k = T_current + 273.15
            T_ambk = T_amb + 273.15
            Q_rad = eps * sigma * A_disc * (T_k**4 - T_ambk**4)

        # net heat flow (Watts)
        Q_net = Q_in - Q_conv - Q_rad

        # dT/dt = Q_net / (m_disc*cp_disc)
        dT = (Q_net / C_total) * dt
        T_new = T_current + dT

        # store
        T_array[i+1] = T_new

    return t_array, T_array

# ------------------- MAIN SCRIPT EXAMPLE ---------------------------------
if __name__ == "__main__":
    # Run the simulation
    t, T = lumped_brake_temp_sim(
        n_brakings=100,
        m_car=290.0,
        I_wheels=3.5,         # example rotational inertia
        p=0.98,
        m_disc=1.2,
        cp_disc=460.0,
        A_disc=0.06,
        eps=0.8,
        v1_kmh=70.0,
        v2_kmh=45.0,
        brake_time=2.5,
        cool_time=3.0,
        T_init=80.0,
        T_amb=25.0,
        h_conv=35.0,
        include_radiation=True,
        dt=0.1
    )

    # Plot the resulting temperature curve
    plt.figure(figsize=(10,6))
    plt.plot(t, T, label="Rotor Temperature (°C)")
    plt.title("Brake Rotor Temperature - Lumped Model (100 Braking Maneuvers)")
    plt.xlabel("Time (s)")
    plt.ylabel("Rotor Temperature (°C)")
    plt.grid(True)
    plt.legend()
    plt.show()

    # Print peak temperature:
    peak_temp = np.max(T)
    print(f"Peak rotor temperature: {peak_temp:.1f} °C at t={t[np.argmax(T)]:.1f} s")
