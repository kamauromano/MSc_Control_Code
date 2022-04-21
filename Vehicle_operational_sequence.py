class uav_operation:
    def __init__(self, mission_depth, max_depth, max_pressure, min_object_distance,
                 interval_distance, mission_location, fluid_density, gps_location_attempt_max,
                 transmit_message_attempts_max):
        # communication values
        self.connection_attempts = 0
        self.gps_location_attempt = 0
        self.gps_location_attempt_max = gps_location_attempt_max
        self.transmit_message_attempts = 0
        self.transmit_message_attempts_max = transmit_message_attempts_max
        # mission values
        self.surface_depth = 0
        self.current_depth = 0
        self.mission_complete = 0
        self.mission_depth = mission_depth
        # safety and object avoidance values
        self.max_depth = max_depth
        self.max_pressure = max_pressure
        self.min_object_distance = min_object_distance
        # location values
        self.current_location_value = []
        self.interval_distance = interval_distance
        self.mission_location = mission_location
        # safety pressure and depth factors
        self.pressure_sf = self.max_depth * 0.05  # 5% of max pressure
        self.depth_sf = self.max_depth * 0.05  # 5% of max depth
        # calculation values
        self.fluid_density = fluid_density
        self.gravity = 9.81

    def connect_control_components(self):
        while (self.connection_attempts < 3):
            try:
                self.make_connection()
                print("connection made")
                self.check_system_readiness()
            except:
                # exceptions will be used to mitigate predetermined failure reasons
                print("connection attempt failed")
        quit()

    def make_connection(self):
        # this will assess the connection between the control components
        pass

    def check_system_readiness(self):
        gps_compass_pass = self.signal_checks_GPS_compass()
        if (gps_compass_pass == 1):
            pass
        else:
            print("vehicle is not ready: check GPS/compass")
            quit()
        motor_pass = self.motor_checks()
        if (motor_pass == 1):
            self.get_location()
        else:
            print("motors not operational")
            quit()

    def signal_checks_GPS_compass(self):
        # this will test the gps and compass
        signal_operational = 1  # (placeholder)
        return signal_operational

    def motor_checks(self):
        # this will test the motor operation
        motors_operational = 1  # (placeholder)
        return motors_operational

    def get_location(self):
        while (self.gps_location_attempt < self.gps_location_attempt_max):
            location_found = self.get_gps_location_of_vehicle()
            if (location_found == 1):
                while (self.transmit_message_attempts < self.transmit_message_attempts_max):
                    message_transmitted = self.transmit_location_to_top_side()
                    if (message_transmitted == 1):
                        self.gps_location_attempt = 0
                        self.transmit_message_attempts = 0
                        if (self.mission_complete == 0):
                            self.measure_object_distance(self.surface_depth)
                        elif (self.mission_complete == 1):
                            print("mission has been completed")
                            self.return_to_base()
                    else:
                        self.transmit_message_attempts += 1
                        continue
                self.sound_error_alarm()
            else:
                self.gps_location_attempt += 1
                print("GPS location attempt failed")
                continue
        print("GPS location could not be found")
        print("Aborting mission")
        print("Please refer to previously reported location")
        self.sound_error_alarm()

    def get_gps_location_of_vehicle(self):
        # this will utilize the GPS module
        # this is the value associated with the coordinate location
        self.current_location_value = [27.2046, 77.4977]  # (placeholder)
        # once location is found
        got_location = 1
        return got_location

    def transmit_location_to_top_side(self):
        # this will utilize the radio trasmitter
        # once location is transmitted
        transmit = 1
        return transmit

    def measure_object_distance(self, current_depth):
        if (self.mission_complete == 1):
            self.resurface(current_depth)
        # this will utilize the sonar
        distance_detected = 20  # (placeholder)
        if (distance_detected > self.min_object_distance):
            self.maneuver_object(current_depth)
        else:
            self.measure_pressure()

    def return_to_base(self):
        self.motion_sequence('return', 0)

    def sound_error_alarm(self):
        # this will utilize a sound device to aid in vehicle rescue
        pass

    def resurface(self, current_depth):
        self.motion_sequence('resurface', current_depth)

    def maneuver_object(self, current_depth):
        # this will utilize sonar readings to maneuver away from the object
        self.measure_object_distance(current_depth)

    def measure_pressure(self):
        pressure_depth = self.pressure_check()
        vehicle_pressure = pressure_depth[0]
        vehicle_depth = pressure_depth[1]
        if ((vehicle_pressure >= (self.max_pressure - self.pressure_sf)) or (vehicle_depth >= (self.max_depth - self.depth_sf))):
            max_depth_reached = 1
        elif ((vehicle_pressure < (self.max_pressure - self.pressure_sf)) and (vehicle_depth < (self.max_depth - self.depth_sf))):
            max_depth_reached = 0
        if ((vehicle_depth >= self.mission_depth) or (max_depth_reached == 1)):
            self.maintain_depth(vehicle_depth)
        elif ((max_depth_reached == 0) and (vehicle_depth < self.mission_depth)):
            self.dive(vehicle_depth)

    def motion_sequence(self, mode, current_depth):
        if (mode == 'dive'):
            # the motion sequence for diving goes here
            self.measure_pressure()
        elif (mode == 'steady'):
            # the motion sequence for maintaining depth goes here
            distance = self.measure_travelled_distance()
            # check if vehicle has reached interval distance or mision location
            if ((distance < self.interval_distance) and (self.at_mission_location() == 0)):
                self.measure_object_distance(current_depth)
            elif ((distance >= self.interval_distance) and (self.at_mission_location() == 0)):
                self.resurface(current_depth)
            elif ((distance < self.interval_distance) and (self.at_mission_location() == 1)):
                self.perform_task(current_depth)
        elif (mode == 'resurface'):
            # the motion sequence for resurfacing goes here
            self.get_location()
        elif(mode == 'return'):
            # the motion sequence to return to start locatrion goes here
            pass

    def pressure_check(self):
        # this will utilize the pressure sensor reading
        pressure_reading = 100000  # (placeholder)
        self.current_depth = pressure_reading / \
            (self.fluid_density * self.gravity)
        return pressure_reading, self.current_depth

    def maintain_depth(self, current_depth):
        self.motion_sequence('steady', current_depth)

    def dive(self, current_depth):
        error_return = self.error_check()
        error_detected = error_return[0]
        error_number = error_return[1]
        if (error_detected == 0):
            self.motion_sequence('dive', current_depth)
        elif (error_detected == 1):
            self.mitigate_error(current_depth, error_number)

    def at_mission_location(self):
        # this will check to see if the original distance + distance travelled is near mission location
        mission_found = 1  # (placeholder)
        return mission_found

    def measure_travelled_distance(self):
        # this will utilize the reading from the PX4
        travelled_distance = 10  # (placeholder)
        return travelled_distance

    def perform_task(self, current_depth):
        error = self.error_check()
        if (error[0] == 1):
            self.mitigate_error(current_depth, error[1])
        else:
            # this is where the mission is initialized and completed
            self.measure_object_distance(current_depth)

    def error_check(self):
        # error checking goes here
        error_detected = 0
        if (error_detected == 1):
            error = 1
            # relevant error number would be sent
            error_no = 1  # (placeholder)
        elif (error_detected == 0):
            error = 0
            error_no = 0
        return error, error_no

    def mitigate_error(self, current_depth, error_number):
        # this will host possible error mitigations
        if (error_number == 1):
            # error solution for first error type
            error_mitigated = 1  # (placeholder)
            if (error_mitigated == 1):
                self.dive(current_depth)
            else:
                self.resurface(current_depth)
