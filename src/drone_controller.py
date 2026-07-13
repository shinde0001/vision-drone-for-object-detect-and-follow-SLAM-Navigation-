import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError
import math

class DroneController:
    def __init__(self, system_address="udp://:14540"):
        self.system_address = system_address
        self.drone = System()
        self.connected = False
        self.telemetry_task = None
        self.reconnect_task = None
        
        self.telemetry = {
            "altitude": 0.0,
            "speed": 0.0,
            "battery": 100,
            "armed": False,
            "flight_mode": "UNKNOWN"
        }
        
    async def connect(self):
        self.reconnect_task = asyncio.create_task(self._reconnect_loop())
        
    async def _reconnect_loop(self):
        while True:
            if not self.connected:
                print(f"Connecting to drone at {self.system_address}...")
                try:
                    # Reset System instance to ensure clean gRPC state
                    self.drone = System()
                    await self.drone.connect(system_address=self.system_address)
                    
                    async for state in self.drone.core.connection_state():
                        if state.is_connected:
                            print("Drone connected successfully!")
                            self.connected = True
                            break
                            
                    # Start telemetry tasks
                    if self.telemetry_task and not self.telemetry_task.done():
                        self.telemetry_task.cancel()
                    self.telemetry_task = asyncio.create_task(self._monitor_telemetry())
                except Exception as e:
                    print(f"Connection attempt failed: {e}. Retrying in 3 seconds...")
                    self.connected = False
                    await asyncio.sleep(3)
            await asyncio.sleep(2)
        
    async def _monitor_telemetry(self):
        try:
            async def mon_alt():
                async for pos in self.drone.telemetry.position():
                    self.telemetry["altitude"] = pos.relative_altitude_m
            async def mon_speed():
                async for vel in self.drone.telemetry.velocity_ned():
                    speed = math.sqrt(vel.north_m_s**2 + vel.east_m_s**2 + vel.down_m_s**2)
                    self.telemetry["speed"] = speed
            async def mon_batt():
                async for batt in self.drone.telemetry.battery():
                    val = batt.remaining_percent
                    self.telemetry["battery"] = int(val * 100) if val <= 1.0 else int(val)
            async def mon_armed():
                async for armed in self.drone.telemetry.armed():
                    self.telemetry["armed"] = armed
            async def mon_mode():
                async for mode in self.drone.telemetry.flight_mode():
                    self.telemetry["flight_mode"] = str(mode)
                    
            await asyncio.gather(mon_alt(), mon_speed(), mon_batt(), mon_armed(), mon_mode())
        except Exception as e:
            print(f"Telemetry stream interrupted: {e}")
            self.connected = False

    def _check_connection(self):
        if not self.connected:
            raise Exception("Drone is disconnected. Please wait for connection...")

    async def arm(self):
        self._check_connection()
        if self.telemetry["armed"]:
            return
        print("Arming drone...")
        try:
            await self.drone.action.arm()
        except Exception as e:
            if "COMMAND_DENIED" in str(e):
                raise Exception("Command Denied. The drone is likely not ready yet (wait for GPS lock) or is in an invalid mode.")
            raise e
        
    async def disarm(self):
        self._check_connection()
        if not self.telemetry["armed"]:
            return
        print("Disarming drone...")
        try:
            await self.drone.action.disarm()
        except Exception as e:
            raise e
        
    async def takeoff(self, altitude=5.0):
        self._check_connection()
        print(f"Taking off to {altitude}m...")
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()
        
    async def land(self):
        self._check_connection()
        print("Landing...")
        try:
            await self.drone.offboard.stop()
        except:
            pass
        await self.drone.action.land()
        
    async def rtl(self):
        self._check_connection()
        print("Returning to launch...")
        try:
            await self.drone.offboard.stop()
        except:
            pass
        await self.drone.action.return_to_launch()
        
    async def start_offboard(self):
        self._check_connection()
        # Must send a setpoint before starting offboard
        print("Starting offboard mode...")
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        try:
            await self.drone.offboard.start()
            return True
        except OffboardError as e:
            print(f"Offboard start failed: {e}")
            return False
            
    async def stop_offboard(self):
        self._check_connection()
        print("Stopping offboard mode...")
        try:
            await self.drone.offboard.stop()
        except:
            pass
            
    async def send_velocity_command(self, forward_m_s, right_m_s, down_m_s, yawspeed_deg_s):
        """Send body-relative velocity command"""
        self._check_connection()
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward_m_s, right_m_s, down_m_s, yawspeed_deg_s)
        )
