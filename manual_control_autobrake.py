#!/usr/bin/env python
"""
Manual CARLA driving with Forward Collision Warning (FCW) and
Automatic Emergency Braking (AEB) based on Time-to-Collision (TTC).

Controls:
    W            - throttle
    S            - brake (hold while stopped to reverse)
    A / D        - steer left / right
    SPACE        - handbrake
    ESC or Q     - quit

Behavior (decision logic lives in aeb_logic.py):
    TTC < 2.6 s  -> FCW: flashing warning on the HUD
    TTC < 1.6 s  -> partial braking (40 %), throttle cut
    TTC < 0.8 s  -> full braking, held until the car is at standstill
TTC = distance / closing speed, using the speed of the object ahead.
"""

import glob
import math
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

from aeb_logic import AebController, AebState

# --------------------------------------------------------------------------------------
# Tunable parameters
# --------------------------------------------------------------------------------------
SENSOR_RANGE = 40.0          # meters - obstacle sensor detection range
DETECTION_TIMEOUT = 0.3      # s - forget a detection if no new event arrives within this time
WINDOW_WIDTH = 960
WINDOW_HEIGHT = 540
VEHICLE_BLUEPRINT = 'vehicle.tesla.model3'

STATE_COLORS = {
    AebState.NORMAL: (60, 255, 60),
    AebState.FCW: (255, 200, 0),
    AebState.PARTIAL_BRAKE: (255, 120, 0),
    AebState.FULL_BRAKE: (255, 40, 40),
}
STATE_TEXT = {
    AebState.NORMAL: 'OK - drive normally',
    AebState.FCW: 'COLLISION WARNING!',
    AebState.PARTIAL_BRAKE: 'AEB: PARTIAL BRAKING',
    AebState.FULL_BRAKE: 'AEB: FULL BRAKING',
}


def speed_along(velocity, forward):
    """Component of a velocity vector along a (unit) forward vector, in m/s."""
    return velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z


class AutoBrakeVehicle:
    def __init__(self, world):
        self.world = world
        self.blueprint_library = world.get_blueprint_library()

        # --- Spawn the vehicle ---
        vehicle_bp = self.blueprint_library.filter(VEHICLE_BLUEPRINT)[0]
        spawn_points = world.get_map().get_spawn_points()
        self.vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])

        # --- Chase camera ---
        camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(WINDOW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(WINDOW_HEIGHT))
        camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
        self.camera = world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        self.surface = None
        self.camera.listen(self._on_camera_image)

        # --- Forward obstacle detector ---
        obstacle_bp = self.blueprint_library.find('sensor.other.obstacle')
        obstacle_bp.set_attribute('distance', str(SENSOR_RANGE))
        obstacle_bp.set_attribute('hit_radius', '0.5')
        obstacle_bp.set_attribute('only_dynamics', 'false')  # also static objects (walls, buildings)
        obstacle_transform = carla.Transform(carla.Location(x=2.2, z=0.5))  # front bumper area
        self.obstacle_sensor = world.spawn_actor(obstacle_bp, obstacle_transform, attach_to=self.vehicle)

        self.obstacle_distance = None
        self.obstacle_actor = None
        self.last_detection_time = 0.0
        self.obstacle_sensor.listen(self._on_obstacle)

        # --- FCW / AEB ---
        self.aeb = AebController()
        self.ego_speed = 0.0
        self.object_speed = 0.0

    # ----------------------------------------------------------------- sensor callbacks
    def _on_camera_image(self, image):
        array = bytearray(image.raw_data)
        self.surface = pygame.image.frombuffer(array, (image.width, image.height), 'BGRA')

    def _on_obstacle(self, event):
        self.obstacle_distance = event.distance
        self.obstacle_actor = event.other_actor
        self.last_detection_time = time.time()

    def _obstacle_is_fresh(self):
        return (time.time() - self.last_detection_time) < DETECTION_TIMEOUT

    # ----------------------------------------------------------------- perception
    def _measure(self):
        """Return (distance or None, ego speed, object speed) along the ego heading."""
        forward = self.vehicle.get_transform().get_forward_vector()
        self.ego_speed = speed_along(self.vehicle.get_velocity(), forward)

        if not self._obstacle_is_fresh() or self.obstacle_distance is None:
            self.object_speed = 0.0
            return None, self.ego_speed, 0.0

        actor = self.obstacle_actor
        try:
            self.object_speed = speed_along(actor.get_velocity(), forward) if actor else 0.0
        except RuntimeError:
            self.object_speed = 0.0   # actor was destroyed meanwhile
        return self.obstacle_distance, self.ego_speed, self.object_speed

    # ----------------------------------------------------------------- control
    def get_control(self, keys):
        control = carla.VehicleControl()

        # --- Driver request ---
        steer = 0.0
        if keys[K_a]:
            steer -= 0.5
        if keys[K_d]:
            steer += 0.5
        control.steer = steer
        control.hand_brake = keys[K_SPACE]

        throttle, brake, reverse = 0.0, 0.0, False
        if keys[K_s]:
            if self.ego_speed > 0.5:
                brake = 1.0                    # moving forward -> S brakes
            else:
                reverse, throttle = True, 0.6  # stopped -> S reverses
        elif keys[K_w]:
            throttle = 0.7
        else:
            brake = 0.3                        # gentle coast-to-stop

        # --- FCW / AEB arbitration ---
        distance, ego_speed, object_speed = self._measure()
        state, aeb_brake = self.aeb.update(distance, max(ego_speed, 0.0), object_speed)

        if aeb_brake > 0.0 and not reverse:
            throttle = 0.0                     # AEB overrides the driver's throttle
            brake = max(brake, aeb_brake)      # driver may always brake harder

        control.throttle = throttle
        control.brake = brake
        control.reverse = reverse
        return control

    def destroy(self):
        for actor in (self.camera, self.obstacle_sensor, self.vehicle):
            if actor is not None:
                actor.destroy()


def draw_hud(display, font, big_font, car):
    state = car.aeb.state
    color = STATE_COLORS[state]

    # Flashing banner for warning / braking
    if state != AebState.NORMAL and int(time.time() * 4) % 2 == 0:
        banner = pygame.Surface((WINDOW_WIDTH, 50), pygame.SRCALPHA)
        banner.fill((*color, 170))
        display.blit(banner, (0, WINDOW_HEIGHT // 2 - 25))
        text = big_font.render(STATE_TEXT[state], True, (255, 255, 255))
        display.blit(text, text.get_rect(center=(WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2)))

    ttc = car.aeb.ttc
    lines = [
        (STATE_TEXT[state], color),
        ('Speed: %5.1f km/h' % (car.ego_speed * 3.6), (255, 255, 255)),
        ('Obstacle: ' + ('%.1f m' % car.obstacle_distance
                         if car._obstacle_is_fresh() and car.obstacle_distance is not None else '--'),
         (255, 255, 255)),
        ('Object speed: %5.1f km/h' % (car.object_speed * 3.6), (255, 255, 255)),
        ('TTC: ' + ('--' if math.isinf(ttc) else '%.2f s' % ttc), color),
    ]
    for i, (txt, col) in enumerate(lines):
        display.blit(font.render(txt, True, col), (10, 10 + 25 * i))


def main():
    pygame.init()
    display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption('CARLA - FCW / AEB (Time-to-Collision)')
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(15.0)
    world = client.get_world()

    car = None
    try:
        car = AutoBrakeVehicle(world)
        font = pygame.font.SysFont('Arial', 20)
        big_font = pygame.font.SysFont('Arial', 32, bold=True)

        running = True
        while running:
            clock.tick(30)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN and event.key in (K_ESCAPE, K_q):
                    running = False

            keys = pygame.key.get_pressed()
            car.vehicle.apply_control(car.get_control(keys))

            if car.surface is not None:
                display.blit(car.surface, (0, 0))
            draw_hud(display, font, big_font, car)
            pygame.display.flip()

    finally:
        if car is not None:
            car.destroy()
        pygame.quit()
        print('done.')


if __name__ == '__main__':
    main()
