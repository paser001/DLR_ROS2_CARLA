import pygame
import time

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise SystemExit("No controller detected")

js = pygame.joystick.Joystick(0)
js.init()

print("Name:", js.get_name())
print("Axes:", js.get_numaxes())
print("Buttons:", js.get_numbuttons())
print("Hats:", js.get_numhats())
print("\nMove one stick / trigger / button at a time...\n")

while True:
    pygame.event.pump()

    axes = [js.get_axis(i) for i in range(js.get_numaxes())]
    buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
    hats = [js.get_hat(i) for i in range(js.get_numhats())]

    print(f"Axes:    {[round(a, 3) for a in axes]}")
    print(f"Buttons: {buttons}")
    print(f"Hats:    {hats}")
    print("-" * 50)

    time.sleep(0.2)