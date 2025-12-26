import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# --------- YOUR PATH ----------
WAYPOINTS_NED = [
    (0.0, 0.0),
    (10.0, 0.0),
    (20.0, 1.0),
    (30.0, 2.0),
    (40.0, 2.0),
    (50.0, 0.0),
]
ALT_AGL = 20.0     # meters AGL
SPEED   = 6.0      # m/s along-path
DT      = 0.05     # seconds (20 Hz setpoints)
YAW_MODE = "track" # "track" to face the path, or "fixed"

def _yaw_deg_from(a, b):
    # heading from point a -> b in degrees (NED frame: x=N, y=E)
    dx, dy = (b[0]-a[0]), (b[1]-a[1])
    return math.degrees(math.atan2(dy, dx))

async def wait_until_armable(drone):
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok and h.is_local_position_ok:
            return

async def goto_along_polyline_offboard(drone, pts, alt_agl, speed, dt, yaw_mode):
    # Seed Offboard: must send one setpoint before start()
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -alt_agl, 0.0))
    await drone.offboard.start()

    try:
        current = (0.0, 0.0)
        yaw_deg = 0.0
        for i in range(len(pts)-1):
            a = pts[i]
            b = pts[i+1]
            seg_dx = b[0]-a[0]
            seg_dy = b[1]-a[1]
            seg_len = math.hypot(seg_dx, seg_dy)
            if seg_len < 1e-6:
                continue
            steps = max(1, int(seg_len/(speed*dt)))
            heading = _yaw_deg_from(a, b) if yaw_mode == "track" else 0.0
            for s in range(steps):
                t = (s+1)/steps
                x = a[0] + seg_dx*t
                y = a[1] + seg_dy*t
                yaw_deg = heading if yaw_mode == "track" else 0.0
                await drone.offboard.set_position_ned(PositionNedYaw(x, y, -alt_agl, yaw_deg))
                await asyncio.sleep(dt)

        # Hold at last point for a second, then RTL
        last = pts[-1]
        await drone.offboard.set_position_ned(PositionNedYaw(last[0], last[1], -alt_agl, yaw_deg))
        await asyncio.sleep(1.0)
    finally:
        try:
            await drone.offboard.stop()
        except OffboardError:
            pass

async def main():
    drone = System()  # default SITL endpoint udp://:14540
    await drone.connect(system_address="udp://:14540")

    print("Waiting for armable…")
    await wait_until_armable(drone)

    print("Arming and taking off…")
    await drone.action.set_takeoff_altitude(ALT_AGL)
    await drone.action.arm()
    await drone.action.takeoff()

    # small settle
    await asyncio.sleep(3.0)

    print("Following custom path in Offboard…")
    await goto_along_polyline_offboard(
        drone, WAYPOINTS_NED, ALT_AGL, SPEED, DT, YAW_MODE
    )

    print("Return-to-launch…")
    await drone.action.return_to_launch()

if __name__ == "__main__":
    asyncio.run(main())
