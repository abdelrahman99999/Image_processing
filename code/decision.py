import numpy as np
import time

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    ## This function can be divided into two parts. Let's imagine a state diagram with the folling states:
    # => Normal states like ['forward movement', 'stop'].
    # => Other special states like ['sample detected', 'stuck in a loop', 'stuck by an obstacle', etc...]
    # We can divide this function into two parts, one for dealing the with the special states, and another for dealing with the normal states.
    
    ## 1. Dealing with special events. Comes first because dealing with special events has higher priority than dealing with normal events.
    # <<== Start of `Handling samples` ==>>
    # If we are near a rock sample, stop all types of movements and send a pickup command.
    if Rover.near_sample:
        print(f"\rNear a sample! {Rover.iteration_counter}\r", end="")
        # Rover.mode = "stop"
        Rover.brake = Rover.brake_set
        Rover.throttle = 0
        Rover.steer = 0
        if Rover.vel == 0 and not Rover.picking_up:
            Rover.send_pickup = True
            Rover.samples_collected += 1
            Rover.rock_detected = False
            Rover.mode = "picked-sample"
            print("")
        return Rover
    
    # <<== End of `Handling samples` ==>>
    
    # Rover.throttle = 0
    # Rover.brake = 0
    
    # <<== Start of `Handling stuck by an obstacle` ==>>
    # Handling when the car is stuck by some obstacle.
    if Rover.mode == 'stuck':
        if Rover.stuck_mode == 'forward':
            print(f"\rStuck by some nasty obstacle! Performing 'forward' action. {Rover.iteration_counter}\r", end="")
            Rover.throttle = 1
            # Rover.steer = 0
            Rover.brake = 0
            Rover.stuck_counter += 1
            
            # If the car is still stuck, repeat again from just moving.
            if Rover.stuck_counter >= 50:
                print("")
                Rover.stuck_mode = 'steer'
                Rover.throttle = 0
                Rover.brake = 0
                # if Rover.nav_angles is not None and len(Rover.nav_angles):
                #     Rover.steer = np.sign(np.mean(Rover.nav_angles * 180/np.pi))*15 # Steer left/right (+ve/-ve).
                # else:
                Rover.steer = -15 # [-15, 15][Rover.iteration_counter%2] # Steer left/right (+ve/-ve).
                Rover.stuck_counter = 0
        
        elif Rover.stuck_mode == 'steer':
            print(f"\rStuck by some obstacle! Performing 'steer' action. {Rover.iteration_counter}\r", end="")
            Rover.throttle = 0
            Rover.brake = 0
            Rover.stuck_counter += 1
            
            # If the car is still stuck, change to moving and steering.
            if Rover.stuck_counter >= 40:
                print("")
                Rover.stuck_mode = 'forward2'
                Rover.throttle = 1
                Rover.steer = np.sign(np.mean(Rover.nav_angles * 180/np.pi))*15
                Rover.brake = 0
                Rover.stuck_counter = 0
        
        elif Rover.stuck_mode == 'forward2':
            print(f"\rStuck by some nasty obstacle! Performing 'forward2' action. {Rover.iteration_counter}\r", end="")
            Rover.throttle = 1
            # Rover.steer = 0
            Rover.brake = 0
            Rover.stuck_counter += 1
            
            # If the car is still stuck, repeat again from just moving.
            if Rover.stuck_counter >= 30:
                print("")
                Rover.stuck_mode = 'backward'
                Rover.throttle = -1
                Rover.brake = 0
                Rover.steer = np.sign(np.mean(Rover.nav_angles * 180/np.pi))*15
                Rover.stuck_counter = 0
        
        elif Rover.stuck_mode == 'backward':
            print(f"\rStuck by some obstacle! Performing 'backward' action. {Rover.iteration_counter}\r", end="")
            Rover.throttle = -1
            Rover.stuck_counter += 1
            
            # If the car is still stuck, change from moving to steering.
            # Two seconds is too long and causes the car to become stuck again.
            if Rover.stuck_counter >= 40:
                print("")
                Rover.stuck_mode = 'forward'
                Rover.throttle = 1
                Rover.steer = np.sign(np.mean(Rover.nav_angles * 180/np.pi))*15
                Rover.stuck_counter = 0
                Rover.brake = 0
        
        if abs(Rover.vel) >= 0.5:
            Rover.stuck_speed_counter += 1
        else:
            Rover.stuck_speed_counter = 0
        
        # If we gain enough speed, then we are finally out of stuck.
        if Rover.stuck_speed_counter > 10:
            print("\nBroke out of the stuck position.")
            Rover.mode = 'forward'
            Rover.stuck_mode = ''
            Rover.throttle = Rover.throttle_set
            Rover.steer = 0
            Rover.brake = 0
            # if Rover.nav_angles is not None and len(Rover.nav_angles):
            #     Rover.steer = np.sign(np.mean(Rover.nav_angles * 180/np.pi))*15 # Steer left/right (+ve/-ve).
            # else:
            #     Rover.steer = [-15, 15][Rover.iteration_counter%2] # Steer left/right (+ve/-ve).
            Rover.stuck_counter = 0
        return Rover
    
    # Checking if the car is stuck by some obstacle while moving.
    if Rover.mode == 'forward' and Rover.vel <= 0.25:
        Rover.stuck_counter += 1
        
        # If the car is stuck for 2 seconds, then it is in a 'stuck' state.
        if Rover.stuck_counter >= 120:
            print("")
            Rover.mode = 'stuck'
            Rover.stuck_mode = 'steer'
            Rover.throttle = 0
            Rover.brake = 0
            # Rover.steer = 0
            # if Rover.nav_angles is not None and len(Rover.nav_angles):
            #     Rover.steer = np.sign(np.mean(Rover.nav_angles * 180/np.pi))*15 # Steer left/right (+ve/-ve).
            # else:
            Rover.steer = -15 #[-15, 15][Rover.iteration_counter%2] # Steer left/right (+ve/-ve).
            Rover.stuck_counter = 0
    else:
        Rover.stuck_counter = 0
    
    # <<== End of `Handling stuck by an obstacle` ==>>
    
    
    if Rover.mode == "picked-sample":
        Rover.brake = 0
        if Rover.picked_sample_counter <= 50:
            Rover.picked_sample_counter += 1
            print(f"\rReturning back some distance. {Rover.iteration_counter}\r", end="")
            Rover.throttle = -Rover.throttle_set
            return Rover
        
        print("")
        Rover.picked_sample_counter = 0
        Rover.throttle = Rover.throttle_set
        
        if abs(Rover.vel) >= 0.3:
            Rover.mode = 'forward'
        elif Rover.nav_angles is not None and len(Rover.nav_angles):
            Rover.steer = np.sign(np.mean(Rover.nav_angles * 180/np.pi))*15 # Steer left/right (+ve/-ve).
        else:
            Rover.steer = -15
    
    
    
    # <<== Start of `Handling loops` ==>>
    if Rover.mode == "loop":
        # Stop steering, keep moving for two seconds, then exit the loop state.
        print(f"\rStuck in a loop! Changing to moving forward. {Rover.iteration_counter}\r", end="")
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        Rover.loop_counter += 1
        if len(Rover.rock_angles) > 0:
            Rover.steer = np.clip(np.mean(np.sort(Rover.rock_angles)[int(len(Rover.rock_angles)/2):] * 180/np.pi), -15, 15)
        
        if Rover.loop_counter >= 50:
            print("")
            Rover.mode = 'forward'
            Rover.loop_counter = 0
        return Rover
    
    # Keeping track of how much we have been moving left/right to break loops.
    if abs(Rover.steer) >= 7: # and not (Rover.rock_detected and Rover.vel == 0)
        Rover.steering_counter += 1
        
        # If the car is steering for 250 frames, then it is stuck in a loop.
        if Rover.steering_counter >= 250:
            print("")
            Rover.mode = "loop"
            Rover.steering_counter = 0
            return Rover
    else:
        Rover.steering_counter = 0
    
    # <<== End of `Handling loops` ==>>
    
    
    # <<== Start of `Handling rock samples` ==>>
    # If a rock sample is identified, make sure that the velocity is low but not zero.
    # if len(np.abs(Rover.rock_angles) >= (5*np.pi/180)) > 0:
    Rover.rock_detected = len(Rover.rock_angles) > 0
    rock_angles_mid = int(len(Rover.rock_angles)/2)
    print(f"\rNAs={len(Rover.nav_angles)}, SAs={len(Rover.rock_angles[rock_angles_mid:])}", end=" ")
    if Rover.rock_detected:
        # Direct the car towards the target sample.
        Rover.steer = np.clip(np.mean(np.sort(Rover.rock_angles)[rock_angles_mid:] * 180/np.pi), -15, 15)
        print(f"| SAV={Rover.steer:0.2f}. {int(Rover.iteration_counter)}", end="")
        
        # Ensure enough speed until the car is within reach to a target sample.
        if Rover.vel <= 1:
            Rover.brake = 0
            if abs(Rover.steer) >=15:
                if Rover.vel != 0:
                    Rover.brake = 0.4
                Rover.throttle = 0
            else:
                if Rover.vel <= 0.2:
                    Rover.throttle = 1
                else:
                    Rover.throttle = 0.4
                
        
        # If the car is still far from the target sample, stop throttling and brake to decrease the speed.
        else:
            Rover.throttle = 0
            # if abs(Rover.vel) > 1:
            Rover.brake = 1
            # else:
            #     Rover.brake = 0.8
        
        return Rover
    
    
    # <<== End of `Handling rock samples` ==>>
    
    
    
    ## 2. Dealing with normal events.
    nav_agnels_len = len(Rover.nav_angles)
    if Rover.nav_angles is not None and nav_agnels_len > 3:
        # One way to force the car to go to all the map locations is to always take the same direction (left or right).
        # If we want to force the car to always go to the left, can discard the angles directed to the right.
        # Check if we have vision data to make decisions with
        nav_angles_mid = int(len(Rover.nav_angles)/2)
        Rover.nav_angles = np.sort(Rover.nav_angles)[nav_angles_mid:]
        
        # Completely discarding the angles directed to the right seems to have some downsides, so let's just take a small subset of them.
        # sorted_nav_angles = np.sort(Rover.nav_angles)
        # Rover.nav_angles = sorted_nav_angles[nav_angles_mid:]
        # Rover.nav_angles = np.concatenate((sorted_nav_angles[:int(nav_angles_mid*0.2)], sorted_nav_angles[nav_angles_mid:]))
        # Rover.nav_angles = np.concatenate((sorted_nav_angles[int(nav_angles_mid*0.15):int(nav_angles_mid*0.55)], np.sort(Rover.nav_angles)[nav_angles_mid:]))
        
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if nav_agnels_len >= Rover.stop_forward:  
                print(f"Normal forward movement. {Rover.iteration_counter}\r", end="")
                
                # If mode is forward, navigable terrain looks good and velocity is below max, then throttle.
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif nav_agnels_len < Rover.stop_forward:
                print("Dead end!")
                
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
        
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if abs(Rover.vel) > 0.2:
                print(f"Stopping... {Rover.iteration_counter}\r", end="")
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Since now we are in the stop mode, check if there is a nearby sample to collect (Not necessary as we have already dealt with this earlier).
                if Rover.rock_detected:
                    print(f"Approaching sample, adjusting speed... {Rover.iteration_counter}\r", end="")
                    # Move towards the rock sample. Don't exit the stop mode to ensure low velocity.
                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                
                # Now we're stopped and we have vision data to see if there's a path forward
                elif nav_agnels_len < Rover.go_forward:
                    print(f"Stopped and can't go forward. {Rover.iteration_counter}\r", end="")
                    # Release the brake to allow turning
                    Rover.brake = 0
                    Rover.throttle = 0 # Rover.throttle_set
                    
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    if abs(Rover.vel) > 0.2:
                        Rover.brake = 1
                        Rover.steer = 0
                    else:
                        Rover.brake = 0
                        Rover.steer = -15 #np.sign(np.mean(Rover.nav_angles * 180/np.pi))*15 # Could be more clever here about which way to turn
                    
                    # if len(Rover.nav_angles) > 100:
                    #     Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    # else:
                        # Rover.steer = -15
                
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif nav_agnels_len >= Rover.go_forward:
                    print("\nStopped but can go forward.")
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        # print(f"No enough angles data received: {len(Rover.nav_angles)} angles. Enter stuck mode.\r")
        # print("Entering 'stuck' mode to resolve the current situation.")
        # Rover.mode = 'stuck'
        # Rover.stuck_mode = 'forward'
        # Rover.throttle = 1
        # Rover.steer = 0
        
        print(f"No enough angles data received: {nav_agnels_len} angles. Steering... ({Rover.iteration_counter})\r", end="")
        Rover.throttle = 0
        # if Rover.nav_angles is not None and len(Rover.nav_angles):
        #     Rover.steer = np.sign(np.mean(Rover.nav_angles * 180/np.pi))*15 # Steer left/right (+ve/-ve).
        # else:
        #     Rover.steer = [-15, 15][Rover.iteration_counter%2] # Steer left/right (+ve/-ve).
        Rover.stuck_counter = 0
        if abs(Rover.vel) > 0.2:
            Rover.brake = 1
            Rover.steer = 0
        else:
            Rover.brake = 0
            Rover.steer = -15
        # Rover.throttle = Rover.throttle_set
        # Rover.steer = 0
    
    return Rover
