import pygame

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
print(f"Controllers found: {count}")

if count == 0:
    raise SystemExit("No controller detected")

js = pygame.joystick.Joystick(0)
js.init()

print("Name:", js.get_name())
print("Axes:", js.get_numaxes())
print("Buttons:", js.get_numbuttons())
print("Hats:", js.get_numhats())

while True:
    pygame.event.pump()

    lx = js.get_axis(0)   # left stick X
    ly = js.get_axis(1)   # left stick Y
    rx = js.get_axis(3)   # often right stick X on Xbox-style mapping
    ry = js.get_axis(4)   # often right stick Y
    lt = js.get_axis(2)   # often left trigger
    rt = js.get_axis(5)   # often right trigger

    a = js.get_button(0)
    b = js.get_button(1)
    x = js.get_button(2)
    y = js.get_button(3)

    print(
        f"LX={lx:.2f} LY={ly:.2f} RX={rx:.2f} RY={ry:.2f} "
        f"LT={lt:.2f} RT={rt:.2f} A={a} B={b} X={x} Y={y}",
        end="\r"
    )