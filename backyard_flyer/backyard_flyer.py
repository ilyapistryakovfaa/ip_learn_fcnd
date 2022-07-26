import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

target_altitude=3.0

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()


    def velocity_callback(self):
        if self.flight_state==States.LANDING:
            if self.global_position[2]-self.global_home[2]<.1:
                if(abs(self.local_position[2]))<.01:
                    self.disarming_transition() #disarm if under .01

    def state_callback(self):
        if self.in_mission:
            if self.flight_state==States.MANUAL:
                self.arming_transition()
            elif self.flight_state==States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self.flight_state==States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition() #if not already self armed and not already self guided, go to manual transition

    def calculate_box(self):
        print("Calc Box")
        hght=target_altitude
        c1=12.0
        c2=0.0
        return [  [c1, c2, hght] , [c1, c1, hght],[c2, c1, hght],[c2, c2, hght] ]

    def arming_transition(self):
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])
        self.flight_state=States.ARMING
        print("arming transition")


    def takeoff_transition(self):
        self.target_position[2]=target_altitude
        self.takeoff(target_altitude)
        self.flight_state=States.TAKEOFF
        print("takeoff transition")

    def waypoint_transition(self):
        self.target_position=self.all_waypoints.pop(0)
        print('Position' , self.target_position)
        self.cmd_position(self.target_position[0] , self.target_position[1], self.target_position[2], 0.0)
        self.flight_state=States.WAYPOINT
        print("waypoint transition")

    def landing_transition(self):
        self.land()
        self.flight_state=States.LANDING
        print("landing transition")

    def disarming_transition(self):
        self.disarm()
        self.release_control()
        self.flight_state=States.DISARMING
        print("disarm transition")

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        #self.connection.start()
        super().start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    conn=MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone=BackyardFlyer(conn)
    time.sleep(2)
    drone.start()