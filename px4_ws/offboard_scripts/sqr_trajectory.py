import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

ALT_AGL = 15.0   # altitude in meters
SIDE = 40.0      # side length in meters
HOVER_TIME = 10   # seconds to wait at each corner

# Waypoints: (N, E, Yaw)
SQUARE_CORNERS = [
    (0.0, 0.0,   0.0),   # Start (facing north)
    (SIDE, 0.0,  90.0),  # East
    (SIDE, SIDE, 180.0), # South
    (0.0, SIDE,  270.0), # West
    (0.0, 0.0,   0.0)    # Back to Start, face North
]


async def wait_until_ready(drone):
    async for health in drone.telemetry.health():
        if health.is_local_position_ok and health.is_global_position_ok:
            print("-- Drone ready for Offboard")
            return


async def fly_square_with_stops(drone):
    print("-- Seeding Offboard setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -ALT_AGL, 0.0))

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"❌ Offboard start failed: {e._result.result}")
        await drone.action.land()
        return

    print("-- Starting square patrol")

    # Visit each corner
    for corner in SQUARE_CORNERS:
        north, east, yaw = corner
        print(f"➡️ Moving to N={north}, E={east}, Yaw={yaw}")

        # Go to the corner and align yaw
        await drone.offboard.set_position_ned(PositionNedYaw(north, east, -ALT_AGL, yaw))

        # Wait at the corner
        await asyncio.sleep(HOVER_TIME)
        print("-- Holding at corner")

    print("-- Square complete, landing")
    await drone.action.land()


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("-- Waiting for drone to connect")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone")
            break

    await wait_until_ready(drone)

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(HOVER_TIME)  # Hover after takeoff

    await fly_square_with_stops(drone)


if __name__ == "__main__":
    asyncio.run(main())
