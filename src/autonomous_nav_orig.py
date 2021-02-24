import glob
import os
import sys
import pandas as pd 
import time
import math
import numpy as np
import controller




try:
    sys.path.append(glob.glob('./PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla





def find_route(start ,destination):
    
    """
    This function finds the shortest distance between the two waypoints start and destination. 
    Then returns a list of all the waypoints in between
    
    """
    
    
    
    current=start
    
    
    
    last_in_current=start.next_until_lane_end(1)
    plan=[]
    while True:
        
        d=[]
        for i in range(len(last_in_current)):
            dist = math.sqrt( (last_in_current[i].transform.location.x - destination.transform.location.x)**2 + (last_in_current[i].transform.location.y - destination.transform.location.y)**2 )
            d.append(dist)
        index_min = np.argmin(d)
        current=last_in_current[index_min]
        current=current.next(0.4)
        last_in_current=current[0].next_until_lane_end(1)
        plan.append(current[0])
        dist=find_dist(current[0] ,destination )
        if dist<20:
            print("We are damn close now time to exit")
            break
            
    return plan



def optimize_plan(plan):
    """
    This function optimizes the route plan passed to it. it checks if two waypoints are more than 45 meter away.
    if there are such waypoints in our route where the distance is more than 45 meter it finds more waypoints between
    such waypoints and creates a new rout _plan
    
    """
    
    
    
    
    last=len(plan)-1
    distance=find_dist(plan[last-1],plan[last-2])
    updated_plan=[]
    for i in range(1,len(plan)):
        distance_bw_wps=find_dist(plan[i],plan[i-1])
        updated_plan.append(plan[i-1])
    
   
        if distance_bw_wps>45:
            new=distance_bw_wps-40
            j=int(new/5)
        
            wp=plan[i-1]
            for w in range(0,j):
                
                wp=wp.next(j)[0]
                distance=find_dist(wp,plan[last])
                if distance<3:
                    print("HI")
                    break
                
                
                if(wp.lane_id != plan[i-1].lane_id):
                    print("Getting Out of the hand ")
                    break
                updated_plan.append(wp)
    updated_plan.append(plan[len(plan)-1])
    return updated_plan






def spawn_vehicle(spawnPoint=carla.Transform(carla.Location(x=27.607,y=3.68402,z=0.02 ),carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))):
    
    """
    
    This function spawn vehicles in the given spawn points. If no spawn 
    point is provided it spawns vehicle in this 
    position x=27.607,y=3.68402,z=0.02
    """
    
    spawnPoint=spawnPoint
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.filter('model3')[0]
    vehicle = world.spawn_actor(bp, spawnPoint)
    return vehicle
    

def setup_PID(vehicle):
    
    
    """
    This function creates a PID controller for the vehicle passed to it 
    """
    
    
    args_lateral_dict = {
            'K_P': 1.95,
            'K_D': 0.2,
            'K_I': 0.07

            ,'dt': 1.0 / 10.0
            }

    args_long_dict = {
            'K_P': 1,
            'K_D': 0.0,
            'K_I': 0.75
            ,'dt': 1.0 / 10.0
            }

    PID=controller.VehiclePIDController(vehicle,args_lateral=args_lateral_dict,args_longitudinal=args_long_dict)
    
    return PID




    
def drive_through_plan(planned_route,vehicle,speed,PID):
    """
    This function drives throught the planned_route with the speed passed in the argument
    
    """
    
    i=0
    target=planned_route[0]
    while True:
        vehicle_loc= vehicle.get_location()
        distance_v =find_dist_veh(vehicle_loc,target)
        control = PID.run_step(speed,target)
        vehicle.apply_control(control)
        
        
        if i==(len(planned_route)-1):
            print("last waypoint reached")
            break 
        
        
        if (distance_v<1.5):
            control = PID.run_step(speed,target)
            vehicle.apply_control(control)
            i=i+1
            target=planned_route[i]
            

    control = PID.run_step(0,planned_route[len(planned_route)-1])
    vehicle.apply_control(control)
                
    
    
    
    





def find_dist(start ,end ):
    dist = math.sqrt( (start.transform.location.x - end.transform.location.x)**2 + (start.transform.location.y - end.transform.location.y)**2 )

    return dist



def find_dist_veh(vehicle_loc,target):
    dist = math.sqrt( (target.transform.location.x - vehicle_loc.x)**2 + (target.transform.location.y - vehicle_loc.y)**2 )
    
    return dist
    

    
    

    

    
    




 
client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
world = client.get_world()



start=world.get_map().get_waypoint(carla.Location(x=27.607,y=3.68402,z=0.02 ))
world.debug.draw_point(start.transform.location,size=0.9)



end= world.get_map().get_waypoint(carla.Location(x=203.4,y=183.72,z=2.343))
world.debug.draw_point(end.transform.location,size=0.9)


#Spawning Vehicle 
vehicle=spawn_vehicle()
PID=setup_PID(vehicle)


# Setting Initial Plan Route 
plan=find_route(start ,end)
world.debug.draw_point(plan[len(plan)-1].transform.location,size=0.8)   

o_plan=optimize_plan(plan) 
#world.debug.draw_point(o_plan[len(o_plan)-1].transform.location,size=0.8)   
speed=60
drive_through_plan(o_plan,vehicle,speed,PID)






"""
for i in range(0 ,len(o_plan)):
    world.debug.draw_point(o_plan[i].transform.location,size=0.1)     
"""