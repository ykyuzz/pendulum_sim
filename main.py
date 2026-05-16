import matplotlib.pyplot as plt
import matplotlib.animation as animation
import scipy
import numpy as np

def pendulum_motion(weight_mass1, 
                    weight_mass2, 
                    weight_stick1, 
                    weight_stick2,
                    length,
                    base_acceleration,
                    base_velocity,
                    gravitational_acceleration, 
                    theta1, 
                    theta2, 
                    theta1_velocity, 
                    theta2_velocity):
    total_weight = weight_mass1 + weight_mass2 + weight_stick1 + weight_stick2
    coef_matrix = np.zeros((2, 2))
    const_vector = np.zeros(2)

    #coef_matrix[0, 0] = np.cos(theta1) * (weight_stick1 / 2 + weight_mass1 + weight_mass2)
    #coef_matrix[0, 1] = np.cos(theta2) * (weight_stick2 / 2 + weight_mass2)
    coef_matrix[0, 0] = length ** 2 * (total_weight - (3 * weight_stick1) / 4)
    coef_matrix[0, 1] = length ** 2 * np.cos(theta1 - theta2) * (weight_stick2 + weight_mass1)
    coef_matrix[1, 0] = length ** 2 * np.cos(theta1- theta2) * (weight_stick2 + weight_mass2)
    coef_matrix[1, 1] = length ** 2 * (weight_stick2 / 4 + weight_mass2)

    #const_vector[0] = -base_acceleration * total_weight / length + theta1_velocity**2 * np.sin(theta1) * (weight_stick1 / 2 + weight_mass1 + weight_mass2) + theta2_velocity ** 2 * np.sin(theta2) * (weight_stick2 / 2 + weight_mass2)
    const_vector[0] = -base_acceleration * length * np.cos(theta1) * (total_weight - weight_stick1 / 2) + base_velocity * theta1_velocity * np.sin(theta1) * (total_weight - weight_stick1) + 2 * theta1_velocity * theta2_velocity * np.sin(theta1 - theta2) * (weight_stick2 + weight_mass2) - theta2_velocity ** 2 * length ** 2 * np.sin(theta1 - theta2) * (weight_stick2 + weight_mass2) - length * gravitational_acceleration * np.sin(theta1) * (total_weight - weight_stick1 / 2)
    const_vector[1] = -base_acceleration * length* np.cos(theta2) * (weight_stick2 / 2 + weight_mass2) + length ** 2 * theta1_velocity ** 2 * np.sin(theta1 - theta2) * (weight_stick2 + weight_mass2)

    return np.linalg.solve(coef_matrix, const_vector)

def ode_system(t, y, params):
    theta1, theta2, theta1_velocity, theta2_velocity = y
    weight_mass1, weight_mass2, weight_stick1, weight_stick2, length, base_acceleration, base_velocity, gravitational_acceleration = params
    theta1_acceleration, theta2_acceleration = pendulum_motion(weight_mass1, weight_mass2, weight_stick1, weight_stick2, length, base_acceleration, base_velocity, gravitational_acceleration, theta1, theta2, theta1_velocity, theta2_velocity)
    return [theta1_velocity, theta2_velocity, theta1_acceleration, theta2_acceleration]

if __name__ == "__main__":
    step = 10000
    max_time = 10
    times = np.linspace(0, max_time, step)

    weight_mass1 = 0.01
    weight_mass2 = 2
    weight_stick1 = 0.001
    weight_stick2 = 0.001
    length = 1

    base_acceleration = 0
    base_velocity = 0
    gravitational_acceleration = 9.81

    initial_theta1 = 0
    initial_theta2 = np.pi / 2
    theta1_velocity = 0
    theta2_velocity = 0

    y = [initial_theta1, initial_theta2, theta1_velocity, theta2_velocity]
    params = [weight_mass1, weight_mass2, weight_stick1, weight_stick2, length, base_acceleration, base_velocity, gravitational_acceleration]
    solution = scipy.integrate.solve_ivp(ode_system, [0, max_time], y, t_eval=times, args=(params,))
    theta1_values = solution.y[0]
    theta2_values = solution.y[1]