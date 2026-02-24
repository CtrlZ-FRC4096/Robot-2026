class LookupTableAll:
    def __init__(self):
        self.data = []

    def add_entry(self, distance, launch_vel, launch_angle, time_of_flight):
        self.data.append((distance, launch_vel, launch_angle, time_of_flight))

    def interpolate(self, distance):
        for i in range(len(self.data) - 1):
            if self.data[i][0] <= distance <= self.data[i + 1][0]:
                d1, vel1, angle1, tof1 = self.data[i]
                d2, vel2, angle2, tof2 = self.data[i + 1]
                
                vel = vel1 + (vel2 - vel1) * (distance - d1) / (d2 - d1)
                angle = angle1 + (angle2 - angle1) * (distance - d1) / (d2 - d1)
                tof = tof1 + (tof2 - tof1) * (distance - d1) / (d2 - d1)

                return vel, angle, tof
        
        return (self.data[-1][1], self.data[-1][2], self.data[-1][3])



class LookupTableVel:
    def __init__(self):
        self.data = []

    def add_entry(self, launch_vel, flywheel_speed):
        self.data.append((launch_vel, flywheel_speed))

    def interpolate(self, launch_vel):
        for i in range(len(self.data) - 1):
            if self.data[i][0] <= launch_vel <= self.data[i + 1][0]:
                l_vel1, fly_speed1 = self.data[i]
                l_vel2, fly_speed2 = self.data[i + 1]
                
                fly_speed = fly_speed1 + (fly_speed2 - fly_speed1) * (launch_vel - l_vel1) / (l_vel2 - l_vel1)

                return fly_speed
        
        return self.data[-1][1]
    

class LookupTableAngle:
    def __init__(self):
        self.data = []

    def add_entry(self, launch_angle, hood_angle):
        self.data.append((launch_angle, hood_angle))

    def interpolate(self, launch_angle):
        for i in range(len(self.data) - 1):
            if self.data[i][0] <= launch_angle <= self.data[i + 1][0]:
                l_angle1, h_angle1 = self.data[i]
                l_angle2, h_angle2 = self.data[i + 1]
                
                hood_angle = h_angle1 + (h_angle2 - h_angle1) * (launch_angle - l_angle1) / (l_angle2 - l_angle1)

                return hood_angle
        
        return self.data[-1][1]

