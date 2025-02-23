import numpy as np
import matplotlib.pyplot as plt

def brake_energy(m_car, I_wheels, v1, v2):
    """
    Computes lump-sum kinetic energy difference 
    from speed v1 to v2 (in m/s).
    """
    # Translational KE
    dE_trans = 0.5 * m_car * (v1**2 - v2**2)
    # Rotational KE (assuming wheels rotate at v/r; 
    # for simplicity, treat I_wheels as 'adjusted' so that 
    # it's just based on linear speed).
    dE_rot = 0.5 * I_wheels * (v1**2 - v2**2)
    return dE_trans + dE_rot

def simulate_variable_braking_events(
    events,
    p=0.95,         # fraction of friction heat into rotor
    m_disc=1.0,     # kg
    cp_disc=460.0,  # J/kg-K
    A_disc=0.05,    # m^2
    h=35.0,         # W/m^2-K convection coefficient
    eps=0.8,        # emissivity
    T_init=60.0,    # °C
    T_amb=25.0,     # °C
    dt=0.1,
    include_radiation=True
):
    """
    Simulate lumped rotor temperature changes over a series
    of braking+cooling events with possible variation in speeds/durations.
    
    Each event is a dict with:
      {
        "v_init": float,    # m/s
        "v_final": float,   # m/s
        "t_brake": float,   # s
        "t_cool": float,    # s
        "m_car": float,
        "I_wheels": float
      }
    """
    sigma = 5.670374419e-8  # Stefan-Boltzmann constant
    C_total = m_disc * cp_disc  # total heat capacity in J/K
    T_current = T_init
    
    time_history = []
    temp_history = []
    brake_flag_history = []  # 1 = braking, 0 = cooling
    
    t_accum = 0.0  # track total time

    for i, ev in enumerate(events):
        v1 = ev["v_init"]
        v2 = ev["v_final"]
        t_brake = ev["t_brake"]
        t_cool = ev["t_cool"]
        m_car = ev["m_car"]
        I_wheels = ev["I_wheels"]

        # --- Calculate lumpsum brake energy for this event ---
        dE = brake_energy(m_car, I_wheels, v1, v2)
        dE_rotor = p * dE  # portion that goes into disc

        # Assume constant power input during t_brake
        power_in = dE_rotor / t_brake if t_brake > 0 else 0.0

        # --- BRAKING PHASE ---
        n_steps_brake = int(np.floor(t_brake / dt))
        for step in range(n_steps_brake):
            # Convection
            Q_conv = h * A_disc * ((T_current + 273.15) - (T_amb + 273.15))
            # You can allow Q_conv to go negative if rotor < ambient,
            # or clamp to zero if you prefer.
            
            # Radiation
            Q_rad = 0.0
            if include_radiation:
                T_k = T_current + 273.15
                T_ambk = T_amb + 273.15
                Q_rad = eps * sigma * A_disc * (T_k**4 - T_ambk**4)

            Q_net = power_in - Q_conv - Q_rad
            dT = (Q_net / C_total) * dt
            T_current += dT

            t_accum += dt
            time_history.append(t_accum)
            temp_history.append(T_current)
            brake_flag_history.append(1)  # braking = 1

        # --- COOLING PHASE ---
        n_steps_cool = int(np.floor(t_cool / dt))
        for step in range(n_steps_cool):
            # No friction
            Q_in = 0.0
            Q_conv = h * A_disc * ((T_current + 273.15) - (T_amb + 273.15))
            Q_rad = 0.0
            if include_radiation:
                T_k = T_current + 273.15
                T_ambk = T_amb + 273.15
                Q_rad = eps * sigma * A_disc * (T_k**4 - T_ambk**4)

            Q_net = Q_in - Q_conv - Q_rad
            dT = (Q_net / C_total) * dt
            T_current += dT

            t_accum += dt
            time_history.append(t_accum)
            temp_history.append(T_current)
            brake_flag_history.append(0)  # cooling = 0

    return np.array(time_history), np.array(temp_history), np.array(brake_flag_history)

if __name__ == "__main__":
    np.random.seed(42)

    # EXAMPLE: 5 random events, each with different speeds and durations
    m_car = 220.0
    I_wheels = 3.5

    events = []
    n_events = 100
    for i in range(n_events):
        v_i = np.random.uniform(15, 25)  # m/s
        dv = np.random.uniform(5, 10)
        v_f = max(0, v_i - dv)
        t_brake = np.random.uniform(1, 3)
        t_cool = np.random.uniform(2, 5)
        ev = {
            "v_init": v_i,
            "v_final": v_f,
            "t_brake": t_brake,
            "t_cool": t_cool,
            "m_car": m_car,
            "I_wheels": I_wheels
        }
        events.append(ev)

    # Run simulation
    t, T, brake_flag = simulate_variable_braking_events(
        events,
        p=0.95,
        m_disc=1.2,
        cp_disc=460.0,
        A_disc=0.06,
        h=30.0,
        eps=0.8,
        T_init=60.0,
        T_amb=25.0,
        dt=0.1,
        include_radiation=True
    )

    # --- PLOTTING ---

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8,6), sharex=True)

    # 1) Plot rotor temperature
    ax1.plot(t, T, label='Rotor Temp (°C)', color='red')
    ax1.set_ylabel("Temperature (°C)")
    ax1.set_title("Lumped Brake Model - Variable Events")
    ax1.grid(True)
    ax1.legend(loc='best')

    # 2) Plot the braking flag below
    # We'll create a step-like line: 1 = braking, 0 = cooling
    ax2.step(t, brake_flag, where='post', color='blue')
    ax2.set_ylabel("Braking Flag")
    ax2.set_yticks([0, 1])
    ax2.set_yticklabels(["Cooling", "Braking"])
    ax2.set_xlabel("Time (s)")
    ax2.grid(True)
    ax2.set_ylim(-0.1, 1.1)

    plt.tight_layout()
    plt.show()

    # Print summary
    peak_temp = np.max(T)
    print(f"Peak Rotor Temp: {peak_temp:.1f} °C at t={t[np.argmax(T)]:.1f}s")
