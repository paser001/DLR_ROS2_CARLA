import carla

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    vehicle_blueprints = blueprint_library.filter('vehicle.*')

    print(f"{len(vehicle_blueprints)} vehicle blueprints:\n")

    for bp in vehicle_blueprints:
        print(bp.id)


if __name__ == "__main__":
    main()