#!/usr/bin/env python
"""
Manual CARLA driving with automatic obstacle braking (AEB).

Controls:
    W            - throttle forward (blocked automatically if an obstacle is ahead)
    S            - brake / reverse (always available - use this to back away from an obstacle)
    A / D        - steer left / right
    SPACE        - handbrake
    ESC or Q     - quit

Behavior:
    - An obstacle sensor is mounted on the front of the car.
    - If it detects something (car, pedestrian, wall, building, etc.) closer than
      SAFE_DISTANCE while you are pressing W, the script overrides your input,
      applies full brake, and sets throttle to 0.
    - The car will NOT move forward again, even if you keep pressing W, until you
      press S and reverse away until the obstacle is farther than SAFE_DISTANCE
      (or no longer detected).
"""

import glob
import os
import sys
import time

# --- Make sure the carla .egg is importable if needed (safe to leave even with pip install) ---
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

try:
    import pygame
    from pygame.locals import K_w, K_s, K_a, K_d, K_SPACE, K_ESCAPE, K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure the pygame package is installed')

# --------------------------------------------------------------------------------------
# Tunable parameters
# --------------------------------------------------------------------------------------
SAFE_DISTANCE = 8.0          # meters - if an obstacle is closer than this, auto-brake kicks in
WINDOW_WIDTH = 960
WINDOW_HEIGHT = 540
VEHICLE_BLUEPRINT = 'vehicle.tesla.model3'


class AutoBrakeVehicle:
    def __init__(self, world):
        self.world = world
        self.blueprint_library = world.get_blueprint_library()

        # --- Spawn the vehicle ---
        vehicle_bp = self.blueprint_library.filter(VEHICLE_BLUEPRINT)[0]
        spawn_points = world.get_map().get_spawn_points()
        self.vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])

        # --- Attach an RGB camera so we can actually see while driving ---
        camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(WINDOW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(WINDOW_HEIGHT))
        camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
        self.camera = world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        self.surface = None
        self.camera.listen(self._on_camera_image)

        # --- Attach an obstacle detector facing forward ---
        obstacle_bp = self.blueprint_library.find('sensor.other.obstacle')
        obstacle_bp.set_attribute('distance', '20')       # max detection range (meters)
        obstacle_bp.set_attribute('hit_radius', '0.5')
        obstacle_bp.set_attribute('only_dynamics', 'false')  # detect static objects (buildings/walls) too
        obstacle_transform = carla.Transform(carla.Location(x=2.2, z=0.5))  # front bumper area
        self.obstacle_sensor = world.spawn_actor(obstacle_bp, obstacle_transform, attach_to=self.vehicle)

        self.obstacle_distance = None      # None = nothing detected
        self.last_detection_time = 0.0
        self.obstacle_sensor.listen(self._on_obstacle)

        # --- AEB state ---
        self.blocked = False  # True = forward throttle is locked out until reverse clears it

    def _on_camera_image(self, image):
        array = bytearray(image.raw_data)
        surf = pygame.image.frombuffer(array, (image.width, image.height), 'BGRA')
        self.surface = surf

    def _on_obstacle(self, event):
        # event.distance = distance to the obstacle, event.other_actor = what was hit
        self.obstacle_distance = event.distance
        self.last_detection_time = time.time()

    def _obstacle_is_fresh(self):
        # obstacle events only fire on detection; treat detection as "stale" after a short window
        # so that once the object leaves the sensor's path, we stop treating it as present.
        return (time.time() - self.last_detection_time) < 0.3

    def get_control(self, keys):
        control = carla.VehicleControl()

        throttle_requested = keys[K_w]
        reverse_requested = keys[K_s]

        obstacle_close = self._obstacle_is_fresh() and self.obstacle_distance is not None \
            and self.obstacle_distance < SAFE_DISTANCE

        # Enter blocked state whenever an obstacle is close and the driver is trying to go forward
        if obstacle_close and throttle_requested and not reverse_requested:
            self.blocked = True

        # Clear the blocked state only once the driver has reversed far enough away
        if self.blocked:
            if not obstacle_close:
                self.blocked = False

        # --- Steering (always allowed) ---
        steer = 0.0
        if keys[K_a]:
            steer -= 0.5
        if keys[K_d]:
            steer += 0.5
        control.steer = steer
        control.hand_brake = keys[K_SPACE]

        # --- Throttle / brake / reverse logic ---
        if reverse_requested:
            # Reversing is always allowed, even while "blocked" - this is how you clear the block
            control.reverse = True
            control.throttle = 0.6
            control.brake = 0.0
        elif self.blocked:
            # AEB active: refuse to move forward no matter what, full brake
            control.reverse = False
            control.throttle = 0.0
            control.brake = 1.0
        elif throttle_requested:
            control.reverse = False
            control.throttle = 0.7
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = 0.3  # gentle coast-to-stop brake when no key pressed

        return control

    def destroy(self):
        for actor in (self.camera, self.obstacle_sensor, self.vehicle):
            if actor is not None:
                actor.destroy()


def main():
    pygame.init()
    display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption('CARLA - Manual Drive with Auto-Brake')
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(15.0)
    world = client.get_world()

    car = None
    try:
        car = AutoBrakeVehicle(world)
        font = pygame.font.SysFont('Arial', 20)

        running = True
        while running:
            clock.tick(30)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN and event.key in (K_ESCAPE, K_q):
                    running = False

            keys = pygame.key.get_pressed()
            control = car.get_control(keys)
            car.vehicle.apply_control(control)

            if car.surface is not None:
                display.blit(car.surface, (0, 0))

            # --- HUD text ---
            status_text = 'AUTO-BRAKE ACTIVE - reverse to clear' if car.blocked else 'OK - drive normally'
            dist_text = f'Obstacle distance: {car.obstacle_distance:.1f} m' if (
                car.obstacle_distance is not None and car._obstacle_is_fresh()) else 'Obstacle distance: -- '

            status_surf = font.render(status_text, True, (255, 60, 60) if car.blocked else (60, 255, 60))
            dist_surf = font.render(dist_text, True, (255, 255, 255))
            display.blit(status_surf, (10, 10))
            display.blit(dist_surf, (10, 35))

            pygame.display.flip()

    finally:
        if car is not None:
            car.destroy()
        pygame.quit()
        print('done.')


if __name__ == '__main__':
    main()