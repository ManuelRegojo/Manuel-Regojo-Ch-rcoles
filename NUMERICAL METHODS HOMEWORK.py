import numpy as np
import matplotlib.pyplot as plt

class Planet:
    def __init__(self, mass, radius, initial_position, initial_velocity):
        self.mass = mass
        self.radius = radius
        self.position = np.array(initial_position, dtype=float)
        self.velocity = np.array(initial_velocity, dtype=float)
        self.acceleration = np.zeros(2)

def calculate_gravitational_force(earth, moon, G=6.67430e-11):
    r = moon.position - earth.position
    distance = np.linalg.norm(r)
    force_magnitude = G * (earth.mass * moon.mass) / (distance ** 2)
    force = force_magnitude * (r / distance)
    return force

def update_planet_position_velocity(planet, force, dt):
    planet.acceleration = force / planet.mass
    planet.velocity += planet.acceleration * dt
    planet.position += planet.velocity * dt

def simulate_orbits(planets, num_steps, dt):
    orbits = [[] for _ in range(len(planets))]

    for step in range(num_steps):
        for i in range(len(planets)):
            force = np.zeros(2)
            for j in range(len(planets)):
                if i != j:
                    force += calculate_gravitational_force(planets[i], planets[j])
            update_planet_position_velocity(planets[i], force, dt)
            orbits[i].append(planets[i].position.copy())


    return orbits

def plot_orbits(orbits):
    for i in range(len(orbits)):
        orbit = np.array(orbits[i])
        planet_name= "earth" if i == 0 else "moon"
        plt.plot(orbit[:, 0], orbit[:, 1], label=f'{planet_name.capitalize()}')

    plt.title("Planetary Orbits Simulation")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.legend()
    plt.show()

# Define planets
earth = Planet(mass=5.97e24, radius=6.371e6, initial_position=[0, 0], initial_velocity=[0, 7.66e3])
moon = Planet(mass=7.34e22, radius=1.7371e6, initial_position=[3.84e8, 0], initial_velocity=[0, 1e3])

# Set simulation parameters
num_steps = 10000
dt = 1000

# Simulate orbits
orbits = simulate_orbits([earth, moon], num_steps, dt)

# Plot orbits
plot_orbits(orbits)
