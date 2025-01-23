from find_ports import search_port
import math
import numpy as np
import rclpy
import serial
import pandas as pd
import time
from std_msgs.msg import String
import pymap3d as pm
import csv
from rclpy.node import Node
from std_msgs.msg import String

global latitude
global longitude

latitude, longitude, altitude = 0, 0, 0
ENU_x, ENU_y, ENU_z = 0, 0, 0

#Kood 15.01.2025 seisuga

arduino = serial.Serial(port=search_port("Arduino"), baudrate=115200, timeout=0.1)

with open("/home/tera/rosws2/src/gps_data/gps_data/kaitsmine.csv", "r") as csvfile:
    reader = csv.reader(csvfile, quotechar='|')
    next(reader)
    data = []
    for row in reader:
        x = float(row[0])
        y = float(row[1])
        #z = float(row[2])
        data.append([x, y])
    print(f"Path length is {len(data)} points")

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'tera_teleop', 10)
        timer_period = 0.05 #seconds
        self.timer = self.create_timer(timer_period, self.input)
        print("Starting Stanley teleop")

    def input(self, data):
        msg = String()
        msg.data = (f"{data[0]},{data[1]}")
        self.publisher_.publish(msg)    
        print(f"Msg: ", msg)

class GPS_Subscriber(Node):
    def __init__(self):
        super().__init__('Stanley_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gps_data_tera',
            self.listener_callback,
            10)
        self.subscription
        self.lat0, self.lon0, self.h0 = 58.3428685594, 25.5692475361, 91.357
        self.latitude, self.longitude, self.altitude = 0, 0, 0
        self.ENU_x, self.ENU_y = 0, 0
        
        

    def listener_callback(self, msg):
        #self.get_logger().info('I heard "%s"' %msg.data)
        info = msg.data.split(',')
        self.latitude = float(info[0])
        self.longitude = float(info[1])
        self.altitude = float(info[2])
        self.ENU_x, self.ENU_y, ENU_z = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.lat0, self.lon0, self.h0)


    def get_coordinates(self):
        return self.latitude, self.longitude
        
    
    def get_ENU(self):
        return self.ENU_x, self.ENU_y

class StanleyController():
    def __init__(self):
        self.target_location = []
        self.target_x = 0
        self.target_y = 0
        self.last_x = 0
        self.last_y = 0
        self.xt_err_corr
        self.k = 0.6                        #0.4
        self.Kp = 1.5                       #1.0
        self.v = 1                       #1.5
        self.k_s = 0.5                        #1
        #self.dt = 0.1                       #0.1
        #self.L = 1.0                        #1.0
        self.max_steer = np.deg2rad(40)
        self.steering_command = 0
        self.suber = GPS_Subscriber()
        self.nearest_point_idx = 0
        self.csv_file = open("/home/tera/rosws2/src/gps_data/gps_data/errors.csv", 'w')  # Open CSV file for writing
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Current heading', 'Current position', 'Target position','Heading error', 'Cross-track error', 'Path direction', 'Unclipped steering'])

        #Simulatsioonis: 
        #k = 1
        #v = 2.5
        #k_s = 0.5
    
    #Function to normalize angles between -pi and pi    
    def normalize(self, input):
        if input > np.pi:
            return np.pi
        
        elif input < -np.pi:
            return -np.pi
        else:
            return input
    

    def get_gps_trace(self):
        return data

    def path_yaws(self, x, y):
        yaws = []

        for i in range(len(x) - 1):
            d_x = x[i+1] - x[i]
            d_y = y[i+1] - y[i]
            yaw = np.arctan2(d_y, d_x)
            yaws.append(yaw)
        yaws.append(yaws[-1])
        #print(len(yaws))
        return np.array(yaws)
    
    #Finding the closest track point to the robot
    def get_target_location(self,x, y):
        i = 0
        min_dist = 100000000
        for i in range(len(data)):
            current_min_dist = math.dist(data[i], [x, y])

            if current_min_dist < min_dist:
                min_dist = current_min_dist
                min_dist_idx = i

        return [data[min_dist_idx], data[min_dist_idx - 1], min_dist_idx, min_dist]
    
    def heading_error(self, yaw, path_yaw):
        yaw_diff = path_yaw - yaw
        
        return self.normalize(yaw_diff)

    def xt_error(self, current_position, closest_point, closest_point_distance): # The cross-track error is the distance between the vehicle and the ideal trajectory
        #Cross track error can be calculated as the distance of the point closest to the front axle
        xte_yaw = self.normalize(np.arctan2(current_position[1] - closest_point[1], current_position[0], closest_point[0]))
        
        xte = np.sign(xte_yaw) * closest_point_distance      #If the robot is to the right or to the left of the path.

        #Not using speed coefficient
        return np.arctan2(self.k * xte)

    def steering_control(self, current_position, target_position, current_heading, path_heading, target_position_dist): # The steering control is the sum of the heading error correction and the cross-track error correction. 
        heading_err = self.heading_error(current_heading, path_heading)
        
        xt_err = self.xt_error(current_position, target_position, target_position_dist)
        self.csv_writer.writerow([current_heading, current_position, target_position, heading_err, xt_err, path_heading, np.rad2deg(heading_err + xt_err)])
        return heading_err + xt_err

    def implementation(self):
        #TODO: implement log file writing.
        
        
        #Getting data and updating values:
        #Get path points
        gps_df = pd.read_csv("/home/tera/rosws2/src/gps_data/gps_data/kaitsmine.csv")   
        x = gps_df["Latitude"].values                                       
        y = gps_df["Longitude"].values
        #Get current position of THE CENTRE OF THE ROBOT
        current_position = [self.ENU_x, self.ENU_y]                                     
        #Calculate robot centre point heading PROBABLY NEED TO REPLACE WITH FRONT AXLE HEADING
        current_heading = self.normalize(np.arctan2(current_position[1] - self.last_y, current_position[0] - self.last_x))  
        #Calculate the position of the front axle
        current_position_fr = [[current_position[0] + 0.38 * np.cos(current_heading), current_position[1] + 0.38 * np.sin(current_heading)]]    
        #Get target location values(RETURNS: nearest point, previous nearest point, nearest point index, min distance)
        target_location = self.get_target_location(current_position_fr[0], current_position_fr[1])            
        nearest_point = target_location[0]                                              
        self.nearest_point_idx = target_location[2]
        min_dist = target_location[3]
        path_yaw = self.path_yaws(x, y)                                                 #Get list of headings of each path point
        target_path_heading = self.normalize(path_yaw[self.nearest_point_idx])                            #Get target locations heading
        current_heading_fr = self.normalize()
        
        #print(f"Path yaw len: {len(path_yaw)}")
        self.steering_command = np.rad2deg(np.clip(self.steering_control(current_position_fr, nearest_point, current_heading_fr, target_path_heading, min_dist), -self.max_steer, self.max_steer))
        #Update values of previous position
        self.last_x = current_position[0]
        self.last_y = current_position[1]
        
        #Function to convert the steering values to values for the arduino 
        def map_range(x, in_min, in_max, out_min, out_max):
            return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
        
        self.adj_steering_command = int(map_range(self.steering_command, 45, -45, 0, 660))

        if self.adj_steering_command == None:
            return 0
        else:
            return int(self.adj_steering_command)
            
    def send2arduino(self, lat, lon, x, y):
        self.latitude = lat
        self.longitude = lon
        self.ENU_x = x
        self.ENU_y = y
        steering = self.implementation()
        if self.nearest_point_idx == len(data):
            pub.input([330, 0])
            print("----------------------End of path----------------------")
        else:
            pub.input([steering, 120])
        
        time.sleep(0.2)
        

if __name__ == '__main__':
    rclpy.init(args=None)
    gps = GPS_Subscriber()
    pub =MinimalPublisher()
    stan = StanleyController()
    while True:
        rclpy.spin_once(gps)
        latitude, longitude = gps.get_coordinates()
        x, y = gps.get_ENU()

        stan.send2arduino(latitude, longitude, x, y)