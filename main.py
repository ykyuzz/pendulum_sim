import matplotlib.pyplot as plt
import matplotlib.animation as animation
import scipy
import numpy as np

def pendulum_motion(weight_mass1, 
                    weight_mass2,
                    length,
                    base_acceleration,
                    base_velocity,
                    gravitational_acceleration, 
                    theta1, 
                    theta2, 
                    theta1_velocity, 
                    theta2_velocity):
    coef_matrix = np.zeros((2, 2))
    const_vector = np.zeros(2)

    #coef_matrix[0, 0] = np.cos(theta1) * (weight_stick1 / 2 + weight_mass1 + weight_mass2)
    #coef_matrix[0, 1] = np.cos(theta2) * (weight_stick2 / 2 + weight_mass2)
    coef_matrix[0, 0] = length ** 2 * (weight_mass1 + weight_mass2)
    coef_matrix[0, 1] = length ** 2 * np.cos(theta1 - theta2) * weight_mass2 / 2
    coef_matrix[1, 0] = length ** 2 * np.cos(theta1 - theta2) * weight_mass2 / 2
    coef_matrix[1, 1] = length ** 2 * weight_mass2

    #const_vector[0] = -base_acceleration * total_weight / length + theta1_velocity**2 * np.sin(theta1) * (weight_stick1 / 2 + weight_mass1 + weight_mass2) + theta2_velocity ** 2 * np.sin(theta2) * (weight_stick2 / 2 + weight_mass2)
    const_vector[0] = -base_acceleration * length * weight_mass1 * np.cos(theta1) + (weight_mass1 + weight_mass2) * gravitational_acceleration * length * np.sin(theta1) - weight_mass2 * (length ** 2 * theta2_velocity ** 2 * np.sin(theta1 - theta2) + length * base_acceleration * np.cos(theta1)) / 2
    const_vector[1] = -base_acceleration * length * weight_mass2 * np.cos(theta2) / 2 + weight_mass2 * gravitational_acceleration * length * np.sin(theta2) + weight_mass2 * length ** 2 * theta1_velocity ** 2 * np.sin(theta1 - theta2) / 2
    return np.linalg.solve(coef_matrix, const_vector)

def ode_system(t, y, params):
    theta1, theta2, theta1_velocity, theta2_velocity = y
    weight_mass1, weight_mass2, length, base_acceleration, base_velocity, gravitational_acceleration = params
    theta1_acceleration, theta2_acceleration = pendulum_motion(weight_mass1, weight_mass2, length, base_acceleration, base_velocity, gravitational_acceleration, theta1, theta2, theta1_velocity, theta2_velocity)
    return [theta1_velocity, theta2_velocity, theta1_acceleration, theta2_acceleration]

def controler_mass1(theta1_velocity, theta1, total_theta1):
    #P = 0.3 * (np.pi - theta1)
    #D = -0.2 * (theta1_velocity)
    #I = 0.5 * (total_theta1)
    #return P + D + I
    return 0

def controler_mass2(theta2_velocity, theta2, total_theta2):
    #P = 0.5 * (np.pi - theta2)
    #D = -0.2 * theta2_velocity
    #I = 0.5 * total_theta2
    #return P + D + I
    return 0

if __name__ == "__main__":
    step = 2000
    max_time = 20
    times = np.linspace(0, max_time, step)

    weight_mass1 = 0.1
    weight_mass2 = 0.1
    length = 0.3


    base_acceleration = 0
    base_velocity = 0
    base_x = 0
    gravitational_acceleration = -9.81

    initial_theta1 = np.pi / 180 * 150
    initial_theta2 = np.pi / 180 * -20
    theta1_velocity = 0
    theta2_velocity = 0
    theta1 = initial_theta1
    theta2 = initial_theta2

    total_theta1 = 0
    total_theta2 = 0

    y = [initial_theta1, initial_theta2, theta1_velocity, theta2_velocity]
    params = [weight_mass1, weight_mass2, length, base_acceleration, base_velocity, gravitational_acceleration]
    #solution = scipy.integrate.solve_ivp(ode_system, [0, max_time], y, t_eval=times, args=(params,))


    locus_x = []
    locus_y = []
    locus_theta1 = []
    locus_theta2 = []
    locus_theta1_velocity = []
    locus_theta2_velocity = []
    locus_theta1_total = []
    locus_theta2_total = []
    locus_controler_mass1 = []
    locus_controler_mass2 = []
    locus_basex = []
    locus_basex_velocity = []
    locus_basex_acceleration = []

    ims = []

    trig_time = 1


    for time in times:
        theta1_acceleration, theta2_acceleration = pendulum_motion(weight_mass1, weight_mass2, length, base_acceleration, base_velocity, gravitational_acceleration, theta1, theta2, theta1_velocity, theta2_velocity)
        theta1_velocity += theta1_acceleration * (max_time / step)
        theta2_velocity += theta2_acceleration * (max_time / step)
        theta1 += theta1_velocity * (max_time / step)
        theta2 += theta2_velocity * (max_time / step)
        total_theta1 += (np.pi / 2 - theta1) * (max_time / step)
        total_theta2 += (np.pi / 2 - theta2) * (max_time / step)
        control_input1 = controler_mass1(theta1_velocity, theta1, total_theta1)
        control_input2 = controler_mass2(theta2_velocity, theta2, total_theta2)
        if base_x < -1.8 or base_x > 1.8:
            base_acceleration = -1 * base_x * step / max_time

        elif time < trig_time:
            base_acceleration = time*np.sin(time / trig_time * 10 * np.pi) * 100
        else:
            base_acceleration = control_input1 + control_input2
        base_velocity += base_acceleration * (max_time / step)
        base_x += base_velocity * (max_time / step)

        

        #for animation
        locus_x.append(length * (np.sin(theta1) + np.sin(theta2)) + base_x)
        locus_y.append(-length * (np.cos(theta1) + np.cos(theta2)))
        locus_theta1.append(theta1)
        locus_basex.append(base_x)
        locus_theta2.append(theta2)
        locus_theta1_velocity.append(theta1_velocity)
        locus_theta2_velocity.append(theta2_velocity)
        locus_basex_velocity.append(base_velocity)
        locus_basex_acceleration.append(base_acceleration)
        locus_theta1_total.append(total_theta1)
        locus_theta2_total.append(total_theta2)
        locus_controler_mass1.append(control_input1)
        locus_controler_mass2.append(control_input2)
        stick1 = plt.plot([base_x, base_x + length * np.sin(theta1)], [0, -length * np.cos(theta1)], c="red", linewidth=1)
        stick2 = plt.plot([base_x + length * np.sin(theta1), base_x + length * (np.sin(theta1) + np.sin(theta2))], [-length * np.cos(theta1), -length * (np.cos(theta1) + np.cos(theta2))], c="red", linewidth=1)
        mass1 = plt.plot(base_x + length * np.sin(theta1), -length * np.cos(theta1), c="red", markersize=weight_mass1 * 100)
        mass2 = plt.plot(base_x + length * (np.sin(theta1) + np.sin(theta2)), -length * (np.cos(theta1) + np.cos(theta2)), c="red", markersize=weight_mass2 * 100)
        time_text = plt.text(0.5, 1.05, "t=" + str(time), ha="center", transform=plt.gca().transAxes)
        im = plt.plot(locus_x, locus_y, c="blue", linestyle="--") + stick1 + stick2 + mass1 + mass2 + [time_text]
        ims.append(im)

    animation = animation.ArtistAnimation(plt.gcf(), ims, interval=max_time / step * 10)
    plt.show()

    plt.plot(times, locus_theta1, label="theta1")
    plt.plot(times, locus_theta2, label="theta2")
    plt.plot(times, locus_theta1_velocity, label="theta1_velocity")
    plt.plot(times, locus_theta2_velocity, label="theta2_velocity")
    plt.plot(times, locus_theta1_total, label="theta1_total_error")
    plt.plot(times, locus_theta2_total, label="theta2_total_error")
    plt.plot(times, locus_controler_mass1, label="controler_mass1")
    plt.plot(times, locus_controler_mass2, label="controler_mass2")
    plt.plot(times, locus_basex, label="base_x")
    #plt.plot(times, locus_basex_velocity, label="base_x_velocity")
    #plt.plot(times, locus_basex_acceleration, label="base_x_acceleration", linestyle="--")
    plt.xlabel("Time")
    plt.ylabel("Angle or Base Position")
    plt.legend()
    plt.show()