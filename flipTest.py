import pygame
import numpy as np
import serial
from threading import Thread
from arduinoDataHandler import arduinoData, get_arduino_angular_data, get_arduino_linear_vdata, get_arduino_linear_adata

# Simulation parameters
GRID_SIZE = 100
PARTICLE_COUNT = 300
TIME_STEP = 0.1
GRAVITY = np.array([0, 9.81, 0]) * TIME_STEP
WINDOW_SIZE = 300
PARTICLE_RADIUS = 2
MAX_DEPTH = 75

# Updated physics parameters
VELOCITY_DAMPING = {
    'x': 0.55,  # Less damping for x-axis
    'y': 0.55,  # Less damping for y-axis
    'z': 0.70   # More damping for z-axis due to larger forces
}
MINIMUM_VELOCITY = 0.01
ANGULAR_FORCE_THRESHOLD = {
    'x': 0,   # Threshold for x-axis (~5% of max observed)
    'y': 0,   # Threshold for y-axis (~10% of max observed)
    'z': 0   # Threshold for z-axis (~4% of max observed)
}
FORCE_SCALING = {
    'x': 2,  # Smaller scaling for x-axis
    'y': 2,  # Smaller scaling for y-axis
    'z': 1  # Much smaller scaling for z-axis due to large values
}
COLLISION_DAMPING = 0.1

class ParticleSystem:
    def __init__(self):
        self.particles = np.random.rand(PARTICLE_COUNT, 3) * np.array([GRID_SIZE, GRID_SIZE * 0.5, MAX_DEPTH])
        self.velocities = np.zeros((PARTICLE_COUNT, 3))
        self.velocity_grid = np.zeros((GRID_SIZE, GRID_SIZE, GRID_SIZE, 3))
        self.grid_cells = {}
        self.force_history = []
        self.MAX_HISTORY = 5

    def update_force_history(self, force):
        self.force_history.append(force)
        if len(self.force_history) > self.MAX_HISTORY:
            self.force_history.pop(0)

    def get_smoothed_force(self):
        if not self.force_history:
            return np.zeros(3)
        return np.mean(self.force_history, axis=0)

    def apply_angular_forces(self, angular_data):
        xlv, ylv, zlv = angular_data["xla"], angular_data["yla"], angular_data["zla"]
        
        # Apply thresholds
        xlv = xlv if abs(xlv) > ANGULAR_FORCE_THRESHOLD['x'] else 0
        ylv = ylv if abs(ylv) > ANGULAR_FORCE_THRESHOLD['y'] else 0
        zlv = zlv if abs(zlv) > ANGULAR_FORCE_THRESHOLD['z'] else 0
        
        # Scale forces
        forces = np.array([
            xlv * FORCE_SCALING['x'],
            ylv * FORCE_SCALING['y'],
            zlv * FORCE_SCALING['z']
        ])
        
        # Update and get smoothed forces
        self.update_force_history(forces)
        return self.get_smoothed_force()

    def apply_velocity_damping(self):
        # Apply axis-specific damping
        self.velocities[:, 0] *= VELOCITY_DAMPING['x']  # x-axis
        self.velocities[:, 1] *= VELOCITY_DAMPING['y']  # y-axis
        self.velocities[:, 2] *= VELOCITY_DAMPING['z']  # z-axis
        
        # Apply minimum velocity threshold
        velocity_magnitudes = np.linalg.norm(self.velocities, axis=1)
        too_slow = velocity_magnitudes < MINIMUM_VELOCITY
        self.velocities[too_slow] = 0

    def update(self):
        # Get and apply angular forces
        angular_data = get_arduino_linear_adata()
        angular_forces = self.apply_angular_forces(angular_data)
        
        # Update velocities with smoothed forces
        grid_based_velocity = self.interpolate_velocity()
        velocity_change = grid_based_velocity - self.velocities
        self.velocities += velocity_change
        
        # Apply forces and damping
        self.velocities += GRAVITY + angular_forces
        self.apply_velocity_damping()
        
        # Move particles
        self.particles += self.velocities * TIME_STEP
        
        # Enhanced boundary handling with axis-specific bounce factors
        for i in range(PARTICLE_COUNT):
            for dim in range(3):
                if self.particles[i, dim] < 0 or self.particles[i, dim] > GRID_SIZE - 1:
                    # Apply different bounce factors for each axis
                    bounce_factor = list(VELOCITY_DAMPING.values())[dim]
                    self.velocities[i, dim] *= -bounce_factor
                    self.particles[i, dim] = np.clip(self.particles[i, dim], 0, GRID_SIZE - 1)
        
        # Handle collisions and update grid
        self.resolve_collisions()
        self.update_velocity_grid()
    def get_cell_index(self, position):
        return tuple(np.floor(position / (PARTICLE_RADIUS * 2)).astype(int))

    def update_spatial_hash(self):
        self.grid_cells.clear()
        for i in range(PARTICLE_COUNT):
            cell = self.get_cell_index(self.particles[i])
            if cell not in self.grid_cells:
                self.grid_cells[cell] = []
            self.grid_cells[cell].append(i)

    def get_nearby_particles(self, position):
        cell = self.get_cell_index(position)
        nearby = []
        for x in range(-1, 2):
            for y in range(-1, 2):
                for z in range(-1, 2):
                    neighbor_cell = (cell[0] + x, cell[1] + y, cell[2] + z)
                    if neighbor_cell in self.grid_cells:
                        nearby.extend(self.grid_cells[neighbor_cell])
        return nearby

    def resolve_collisions(self):
        self.update_spatial_hash()
        
        for i in range(PARTICLE_COUNT):
            nearby = self.get_nearby_particles(self.particles[i])
            
            for j in nearby:
                if i == j:
                    continue
                    
                diff = self.particles[i] - self.particles[j]
                dist = np.linalg.norm(diff)
                
                if dist < PARTICLE_RADIUS * 2 and dist > 0:
                    # Collision response
                    direction = diff / dist
                    overlap = (PARTICLE_RADIUS * 2) - dist
                    
                    # Move particles apart
                    self.particles[i] += direction * overlap * 0.5
                    self.particles[j] -= direction * overlap * 0.5
                    
                    # Exchange momentum with damping
                    rel_vel = self.velocities[i] - self.velocities[j]
                    normal_vel = np.dot(rel_vel, direction) * direction
                    
                    self.velocities[i] -= normal_vel * (1 + COLLISION_DAMPING) * 0.5
                    self.velocities[j] += normal_vel * (1 + COLLISION_DAMPING) * 0.5

    def interpolate_velocity(self):
        new_velocities = np.zeros_like(self.particles)
        for i, (x, y, z) in enumerate(self.particles):
            gx = int(x) % GRID_SIZE
            gy = int(y) % GRID_SIZE
            gz = int(z) % GRID_SIZE
            new_velocities[i] = self.velocity_grid[gx, gy, gz]
        return new_velocities

    def update_velocity_grid(self):
        self.velocity_grid.fill(0)
        counts = np.zeros((GRID_SIZE, GRID_SIZE, GRID_SIZE))
        
        for i, (x, y, z) in enumerate(self.particles):
            gx = int(x) % GRID_SIZE
            gy = int(y) % GRID_SIZE
            gz = int(z) % GRID_SIZE
            self.velocity_grid[gx, gy, gz] += self.velocities[i]
            counts[gx, gy, gz] += 1
        
        mask = counts > 0
        self.velocity_grid[mask] /= counts[mask, None]

def get_color(z_pos, z_vel):
    z_normalized = np.clip(z_pos / MAX_DEPTH, 0, 1)
    z_vel_normalized = np.clip(z_vel / 10, -1, 1)
    
    if z_vel_normalized > 0:
        r = 0
        g = int(255 * abs(z_vel_normalized))
        b = int(255 * z_normalized)
    else:
        r = int(255 * abs(z_vel_normalized))
        g = 0
        b = int(255 * z_normalized)
    
    return (int(np.clip(r, 0, 255)), 
            int(np.clip(g, 0, 255)), 
            int(np.clip(b, 0, 255)))

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
clock = pygame.time.Clock()

def render(particle_system):
    screen.fill((0, 0, 0))
    sorted_indices = np.argsort(particle_system.particles[:, 2])
    
    for idx in sorted_indices:
        screen_x = int((particle_system.particles[idx, 0] / GRID_SIZE) * WINDOW_SIZE)
        screen_y = int((particle_system.particles[idx, 1] / GRID_SIZE) * WINDOW_SIZE)
        
        z_factor = 1 + (particle_system.particles[idx, 2] / MAX_DEPTH)
        size = max(1, int(PARTICLE_RADIUS * z_factor))
        
        try:
            color = get_color(particle_system.particles[idx, 2], particle_system.velocities[idx, 2])
            pygame.draw.circle(screen, color, (screen_x, screen_y), size)
        except ValueError as e:
            print(f"Color error: {e}, Color values: {color}")
            continue
    
    pygame.display.flip()

def main():
    particle_system = ParticleSystem()
    running = True
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        particle_system.update()
        render(particle_system)
        clock.tick(60)
    
    pygame.quit()

if __name__ == "__main__":
    threadA = Thread(target=arduinoData)
    threadA.start()
    main()