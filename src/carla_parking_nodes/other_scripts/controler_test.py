import pygame

pygame.init()
pygame.controller.init()

count = pygame.controller.get_count()
print(f"Controllers found: {count}")

if count == 0:
    raise SystemExit("No controller detected")

ctrl = pygame.controller.Controller(0)
ctrl.init()

print("Name:", ctrl.get_name())

while True:
    pygame.event.pump()

    # Standardized controller axes
    lx = ctrl.get_axis(pygame.CONTROLLER_AXIS_LEFTX)
    ly = ctrl.get_axis(pygame.CONTROLLER_AXIS_LEFTY)
    rx = ctrl.get_axis(pygame.CONTROLLER_AXIS_RIGHTX)
    ry = ctrl.get_axis(pygame.CONTROLLER_AXIS_RIGHTY)
    lt = ctrl.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT)
    rt = ctrl.get_axis(pygame.CONTROLLER_AXIS_TRIGGERRIGHT)

    # Standardized buttons
    a = ctrl.get_button(pygame.CONTROLLER_BUTTON_A)
    b = ctrl.get_button(pygame.CONTROLLER_BUTTON_B)
    x = ctrl.get_button(pygame.CONTROLLER_BUTTON_X)
    y = ctrl.get_button(pygame.CONTROLLER_BUTTON_Y)

    print(
        f"LX={lx:.2f} LY={ly:.2f} RX={rx:.2f} RY={ry:.2f} "
        f"LT={lt:.2f} RT={rt:.2f} A={a} B={b} X={x} Y={y}",
        end="\r",
        flush=True
    )