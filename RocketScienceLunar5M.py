import time
import krpc
import matplotlib.pyplot as plt
import copy
import numpy as np
import sys
import numpy.linalg as la

#this skrip requires MechJeb 2
#Pleas install the MechJeb 2 mod AND install this: https://genhis.github.io/KRPC.MechJeb/installation.html

conn = krpc.connect()
vessel = conn.space_center.active_vessel
sc =  conn.space_center
ap = vessel.auto_pilot
mj = conn.mech_jeb
missionName = vessel.name 

t_list = []
apoapsisTime_list = []
pitch_list = []
throttle_list = []
thrust_list = []
altitude_list = []
gForce_list = []
MRE_gForce_list =[]
MRE_altitude_list = []
MRE_speed_list = []
MRE_time_list = []

stage1_t_list = []
stage1_altitude_list = []
stage1_gForce_list = []
stage1LandingSpeed = []
stage1LandingAltitude = []
stage1LandingThrottle = []
stage1LandingTime = []
stage1LandingT = 0

stage2_t_list = []
stage2_throttle_list = []
stage2_thrust_list = []
stage2_altitude_list = []
stage2_gForce_list = []


#pre flight check

#Flight State
accentPhase = True
cruisePhase = False
insertionPhase = False
orbitPhase = False
orbit2Phase = False
mtoPhase = False
captureBurnPhase = False
munLandingPhase = False
munLiftOffPhase = False
munAccentPhase = False
munCruisePhase = False
munInsertionPhase = False
munRendevouzPhase = False
munDockingPhase = False
munOrbitPhase = False 
ktoPhase = False
mainReentryPhase = False
stage1Phase = False
stage2Phase = False

GravityTurn = False
fairing = False
fineTune = False
stage1SetTime = False
stage1Reentry = False
stage1SlowDownBurn = False
stage1SlowDownBurnEnd = False
stage1Parachute = False
stage1landingBurn = False
stage1landed = False
stage2CorrectionBurn = False
retroBurn = False
stage2Seperation = False
stage2SetTime = False 
stage2Reentry = False
stage2SlowDownBurn = False
stage2ParachuteArmed = False
stage2Parachute = False
stage2landed = False
vesselBehindTarget = False
fineTuneMTO = False
fineTuneKTO = False
munSoi = False
fineTuneCapture = False
alphaTested = False
munGravityTurn = False
munFineTune1 = False
munEndBurn1 = False
munVesselBehindTarget = False
fineTuneMatchSpeed = False
endBurnMatchSpeed = False
kerbinSoi = False
mainReentryPeriapsis = False



t1 = time.time()
t = 0
stage1_t = 0
stage1Vessel = 0
stage2_t = 0
stage2Vessel = 0


mainVessel = conn.space_center.active_vessel

srbs_separated = False
turn_angle = 0

turn_start_altitude = 250
turn_end_altitude = 50000 #decides how agressive the turn is. The lower, the more aressive. 
target_altitude = 100000

# Set up streams for telemetry
obt_frame = vessel.orbit.body.non_rotating_reference_frame
srf_frame = vessel.orbit.body.reference_frame
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
time_to_periapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_periapsis')
vertical_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'vertical_speed')
horizontal_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'horizontal_speed')

stageInWhichSrbDecouple = 8
stage_SRB_decouple_resources = vessel.resources_in_decouple_stage(stage=stageInWhichSrbDecouple, cumulative=False)
srb_fuel = conn.add_stream(stage_SRB_decouple_resources.amount, 'SolidFuel')

conn.space_center.target_body = conn.space_center.bodies['Mun']
target_body = conn.space_center.target_body

def angle_between_vektors(v, w):
    angleRAD = np.arccos(np.dot(v, w)/(np.linalg.norm(v)*np.linalg.norm(w)))
#    angleDEG = np.rad2deg(angleRAD)
    return angleRAD

def vessel_Speed():
    return 2*np.pi/vessel.orbit.period

def targetAngle():
    mu = vessel.orbit.body.gravitational_parameter
    r1 = vessel.orbit.radius
    r2 = target_body.orbit.radius
    travelTime = np.pi * np.sqrt((r1 + r2) ** 3 / (8 * mu))
    angle = 180 - (360 * travelTime) / target_body.orbit.period
    return angle

def targetBodyPos():
    return target_body.orbit.position_at(ut(), vessel.orbit.body.non_rotating_reference_frame)

def vesselPos():
    return sc.active_vessel.orbit.position_at(ut(), sc.active_vessel.orbit.body.non_rotating_reference_frame)

def targetPos():
    return conn.space_center.target_vessel.orbit.position_at(ut(), vessel.orbit.body.non_rotating_reference_frame)

def munPos():
    return mun.position(kerbin.non_rotating_reference_frame)

def sunPos():
    return sun.position(kerbin.non_rotating_reference_frame)

def kerbinPos():
    return kerbin.position(vessel.orbit.body.non_rotating_reference_frame)

def textUpdate():
    try:
        textAltitude.content = 'Altitude: ' + str(round(altitude(), 1))
    except Exception as e:
        print(e)
    
def execute_node():
    print("Executing next maneuver node")
    executor.execute_one_node()
    
    with conn.stream(getattr, executor, "enabled") as enabled:
        enabled.rate = 1 #we don't need a high throughput rate, 1 second is more than enough
        with enabled.condition:
            while enabled():
                enabled.wait()
                
def targetDistance():
    # Get distances
    current = conn.space_center.active_vessel.parts.controlling.docking_port
#    target = conn.space_center.target_docking_port
    target = conn.space_center.target_vessel
    current_position = current.position(target.reference_frame)
    displacement = np.array(current_position)
    distance = la.norm(displacement)
    return distance

def targetSpeed():
    # speeds relative to the target docking port
    current = conn.space_center.active_vessel.parts.controlling.docking_port
    target = conn.space_center.target_vessel
    velocity = current.part.velocity(target.reference_frame)
    speed = la.norm(np.array(velocity))
    return speed

def suicideBurn():
    v = vertical_speed()
    g = vessel.orbit.body.surface_gravity
    m = vessel.mass
    F = vessel.available_thrust
    a = F/m - g
    h = v ** 2 / (2*a)
    return h

def mainReentryData():
    MRE_gForce_list.append(sc.active_vessel.flight().g_force)
    MRE_altitude_list.append(sc.active_vessel.flight().surface_altitude)
    MRE_speed_list.append(sc.active_vessel.flight(sc.active_vessel.orbit.body.reference_frame).speed)
    MRE_time_list.append(MRE_t)


    

#def relativeVelocity():
#    # speeds relative to the target docking port
#    current = conn.space_center.active_vessel.parts.controlling.docking_port
#    try:
#        target = conn.space_center.target_docking_port
#        velocity = current.part.velocity(target.reference_frame)
#    except:
#        try:
#            target = conn.space_center.target_vessel
#            velocity = current.part.velocity(target.reference_frame)
#        except:
#            velocity = (0, 0, 0)
#            pass        
#    velocity = np.array(velocity)
##    speed = la.norm(np.array(velocity))
#    return velocity
#
#def relativeDistance():
#    # Get distances
#    current = conn.space_center.active_vessel.parts.controlling.docking_port
#    try:
#        target = conn.space_center.target_docking_port
#        current_position = current.position(target.reference_frame)
#    except:
#        try:
#            target = conn.space_center.target_vessel
#            current_position = current.position(target.reference_frame)
#        except:
#            current_position = (0, 0, 0)
#            pass   
#    displacement = np.array(current_position)
##    distance = la.norm(displacement)
#    return displacement

#def dockTextUpdate():
#    text1.content = 'Vx: ' + str(round(relativeVelocity()[0], 2))
#    text2.content = 'Vy: ' + str(round(relativeVelocity()[1], 2))
#    text3.content = 'Vz: ' + str(round(relativeVelocity()[2], 2))    


if vessel.situation == conn.space_center.VesselSituation.pre_launch:

    vessel.type = sc.VesselType.ship
    sc.quicksave()

    vessel.control.sas = False
    print("SAS OFF")
    time.sleep(0.3)
    vessel.control.rcs = False
    print("RCS OFF")
    time.sleep(0.3)
    vessel.control.throttle = 1
    print("Throttle 100%")
    time.sleep(0.3)
    
    if srb_fuel() > 0:
        accentPhase = True
        print("Pre flight checks done.")
        time.sleep(0.3)
        print('Go for Lift Off.')
        time.sleep(0.3)
        print("Starting contdown")
        time.sleep(0.3)
        
        #Countdown Sequence
        countdown = ["T -2", "T -1", "Lift Off!"]
        
        print("T -3")
        for i in range(len(countdown)):
            time.sleep(1)
            print(countdown[i])
            
        vessel.control.activate_next_stage()  #10
        vessel.auto_pilot.engage()
        vessel.auto_pilot.target_pitch_and_heading(90, 90)
        print('accentPhase')
        
    else:
        print('Hold!, Hold!, Hold!')
        time.sleep(0.3)
        vessel.control.throttle = 0
        print('Throttle 0%')
        time.sleep(0.3)
        print('Pre flight checks failed.')
        time.sleep(0.3)
        print('Check your staging')
        time.sleep(0.3)
        print('No SRB fuel in Stage', stageInWhichSrbDecouple) 
        time.sleep(0.3)
        sys.exit()

while accentPhase or cruisePhase or insertionPhase or orbitPhase or mtoPhase or captureBurnPhase or munLandingPhase or munLiftOffPhase or munAccentPhase or munCruisePhase or munInsertionPhase or munRendevouzPhase or munDockingPhase or munOrbitPhase or ktoPhase or mainReentryPhase or stage1Phase or stage2Phase:
    
    if accentPhase or cruisePhase or insertionPhase or orbitPhase:
        #rough flight data (not allways working. In sections like "Stage 2 retroburn" no data is recorded, beause of time.sleep() or while loops)
        t_list.append(t)
        t = time.time() - t1
        
        pitch_list.append(vessel.flight().pitch)
        throttle_list.append(vessel.control.throttle)
        thrust_list.append(vessel.thrust)
        altitude_list.append(altitude())
        gForce_list.append(vessel.flight().g_force)
    
        if time_to_apoapsis() < 200:
            apoapsisTime_list.append(time_to_apoapsis())
        else:
            apoapsisTime_list.append(0)
    
    
    if accentPhase:
        targetPitch = 90 * ((50000 - altitude()) / 50000)
        pitchDiff = vessel.flight().pitch - targetPitch
        
        # BECO / Separate SRBs when finished
        if not srbs_separated:
            if srb_fuel() < 161: #in the sepatron 1 are 160 units of srb_fuel left
                print('BECO')
                vessel.control.activate_next_stage() #9
                srbs_separated = True
                print('SRBs separated')
  
        # Gravity turn
        if not GravityTurn:
            if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
                frac = ((altitude() - turn_start_altitude) /
                        (turn_end_altitude - turn_start_altitude))
                new_turn_angle = frac * 90
                if abs(new_turn_angle - turn_angle) > 0.5:
                    turn_angle = new_turn_angle
                    vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)
        
            # Decrease throttle when approaching target apoapsis
            if apoapsis() > target_altitude * 0.9:
                print('Approaching target apoapsis')
                
                # Disable engines when target apoapsis is reached
                vessel.control.throttle = 0.25
                while apoapsis() < target_altitude:
                    pass
                GravityTurn = True
                vessel.auto_pilot.disengage()

        #MECO
        if apoapsis() > target_altitude:
            vessel.control.throttle = 0
            time.sleep(0.5)
            stage1Shallow = vessel.control.activate_next_stage() #8
            stage1 = copy.deepcopy(stage1Shallow)
            #print(stage1)
            print('MECO')
            
            vessel.control.sas = True
            vessel.control.rcs = True
            time.sleep(0.1)
            vessel.control.sas_mode = conn.space_center.SASMode.prograde
            time.sleep(3)
    
            accentPhase = False
            cruisePhase = True
            print('cruisePhase')
                        
    elif cruisePhase:
        
        #SPACE!!!
        if altitude() > 70000:
            #Deploy fairing
            if fairing == False:
                vessel.control.activate_next_stage() #7
                time.sleep(0.5)
                print('Deploy fairing')
                time.sleep(0.5)
                print("SPACE!!!")
                fairing = True
                
            # Plan circularization burn (using vis-viva equation)
            print('Planning circularization burn')
            mu = vessel.orbit.body.gravitational_parameter
            m = vessel.mass
            G = 6.67428e-11
            r = vessel.orbit.apoapsis
            a = vessel.orbit.semi_major_axis
            v1 = np.sqrt((mu + G*m) * ((2./r) - (1./a)))
            v2 = np.sqrt((mu + G*m) * (1./r))
            delta_v = v2 - v1
            print('needed dv:', delta_v)
            node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)
            
            #Aktivate engine for available thrust calculation
            vessel.control.activate_next_stage() #6
            
            # Calculate burn time (using rocket equation)
            F = vessel.available_thrust
            Isp = vessel.specific_impulse * 9.82
            m0 = vessel.mass
            m1 = m0 / np.exp(delta_v/Isp)
            flow_rate = F / Isp
            burn_time = (m0 - m1) / flow_rate
            
            # Wait until burn
            print('Waiting until circularization burn')
            burn_ut = ut() + time_to_apoapsis() - (burn_time/2.)
            lead_time = 6
            conn.space_center.warp_to(burn_ut - lead_time)
            cruisePhase = False
            insertionPhase = True
            print("insertionPhase")
            
            # Orientate ship
            vessel.control.sas_mode = conn.space_center.SASMode.maneuver
                
    elif insertionPhase:

        # Execute burn
        if time_to_apoapsis() < (burn_ut - lead_time):
            if fineTune == False:
                print('Ready to execute burn')
                time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
                while time_to_apoapsis() - (burn_time/2.) > 0:
                    pass
                print('Executing burn')
                vessel.control.throttle = 1.0
                while node.remaining_delta_v > (node.delta_v * 0.01):
                    pass
                fineTune = True
                endBurn = False
                
            elif fineTune == True and endBurn == False:
                print('Fine tuning')
                vessel.control.throttle = 0.05
                vessel.control.rcs = False
                try:
                    vessel.control.sas_mode = conn.space_center.SASMode.maneuver
                except Exception as e:
                    print(e)
                    try:
                        vessel.control.sas_mode = conn.space_center.SASMode.prograde
                    except Exception as e:
                        print(e)
                while node.remaining_delta_v >= 0.1:
                    pass
                
                #SECO 
                vessel.control.throttle = 0.0
                print('SECO')
                node.remove()
                endBurn = True
                insertionPhase = False
                
                stage1Phase = True
                print('stage1Phase')
                
                vessel.control.set_action_group(3, True)
                time.sleep(5)
                print("Solar panels deployed")
                print("Antenna deployed")
                print('switch ot stage 1')
                
                time.sleep(1)
                conn.space_center.active_vessel = stage1[0]
                                
    elif stage1Phase:
        #Stage 1 reentry and landing
        stage1Vessel = stage1[0]
        stage1Altitude = stage1Vessel.flight().mean_altitude
        stage1Altitude = conn.add_stream(getattr, stage1Vessel.flight(), 'mean_altitude')
        
        #rename Vessel
        if stage1Vessel.name == mainVessel.name + ' Probe':
            stage1Vessel.name = mainVessel.name + ' Stage 1'
        
        #stage 1 time
        if stage1SetTime == False:
            stage1_t1 = time.time()
            stage1SetTime = True
        
        stage1_t = time.time() - stage1_t1
        stage1_t_list.append(stage1_t)
        
        #stage 1 rough flight data (not allways working. In sections like "Stage 2 retroburn" no data is recorded, beause of time.sleep() or while loops)
        stage1_altitude_list.append(stage1Altitude())
        stage1_gForce_list.append(stage1Vessel.flight().g_force)
        
        #Time Warp until reentry
        if stage1Altitude() > 75000:
            time.sleep(2) 
            conn.space_center.physics_warp_factor = 3
            while stage1Altitude() > 75000:
                pass
            conn.space_center.physics_warp_factor = 0
    
        if stage1Altitude() < 70000 and stage1Reentry == False and fineTune == True:
            print('start stage 1 reentry')
            stage1Reentry = True
        
        if stage1Reentry and stage1SlowDownBurnEnd == False:
            stage1Vessel.control.sas = True
            stage1Vessel.control.sas_mode = conn.space_center.SASMode.retrograde
        
        #slow down burn
        if stage1Altitude() < 11000 and stage1SlowDownBurnEnd == False:
            stage1Vessel.control.throttle = 1
            print('stage 1 slow down burn')
            stage1SlowDownBurn = True
        
        if stage1SlowDownBurn:
            while stage1Vessel.flight(stage1Vessel.orbit.body.reference_frame).speed > 50:
                pass
            stage1Vessel.control.throttle = 0
            stage1Vessel.control.sas = False
            stage1SlowDownBurn = False
            stage1SlowDownBurnEnd = True
            
        #Parachute
        if stage1Altitude() < 10000 and stage1Parachute == False:
            try:
                for parachutes in conn.space_center.active_vessel.parts.parachutes: #does not allways work.
                     parachutes.deploy()      
            except Exception as e:
                print('!!!!!!!!!!!!!!!!!!!!!!!!!!')
                print('')
                print(e)
                print('')
                print('Something went horribly wrong. Switching to PSPP (Panicly Spacebar Press Procedure).')
                print('')
                print('!!!!!!!!!!!!!!!!!!!!!!!!!!')
                for i in range(5): #spaming spacebar to activate parachutes, because parachutes not allways activate at the first try.
                    stage1Vessel.control.activate_next_stage()
                print('stage 1 parachute deployed (hopefully)')
            else:  
                print('stage 1 parachute deployed (hopefully)')  
            finally:
                stage1Vessel.control.sas = False
                stage1Parachute = True
                
        #wait for landing burn
        if stage1Altitude() < 550:
            while stage1Altitude() > 100:
                conn.space_center.physics_warp_factor = 4
                pass
            conn.space_center.physics_warp_factor = 0
            time.sleep(1)
        
        #landing burn   
        if stage1Altitude() < 50:
            if not stage1landingBurn:
                print('stage 1 landing burn')
                stage1landingBurn = True
                
            try:
                #Switching off four engines for landing burn
                print('Switching off four engines for landing burn')
                for engine in conn.space_center.active_vessel.parts.engines[0:4]:
                    engine.active = False
                
            except Exception as e:
                print('!!!!!!!!!!!!!!!!!!!!!!!!!!')
                print('')
                print(e)
                print('')
                print('Something went horribly wrong. Switching to five enging landing burn.')
                print('')
                print('!!!!!!!!!!!!!!!!!!!!!!!!!!')

                #five engine landing burn (emergency procedure if enging shutt off doesn't work)
                HIGH = 0.2
                LOW = 0.075
                
            else:
                #one engine landing burn (standard procedure)
                HIGH = 0.7 #0.6
                LOW = 0.4 #0.3
                
                if conn.space_center.active_vessel.parts.engines[0].can_shutdown == False:
                    print('Can not shutdown engines for unknowen reasons. Switching to five enging landing burn.')
                    HIGH = 0.2
                    LOW = 0.075

            finally:
                while stage1landingBurn:
                    stage1LandingT += 1
                    stage1LandingTime.append(stage1LandingT)
#                    print('')
#                    print('altitude:', stage1Altitude())
                    stage1LandingAltitude.append(stage1Altitude())
                    speed = stage1Vessel.flight(stage1Vessel.orbit.body.reference_frame).speed
#                    print('speed:', speed)
                    stage1LandingSpeed.append(speed)
                    if speed > 3: #1:
                        stage1Vessel.control.throttle = HIGH
#                        print(stage1Vessel.control.throttle)
                        stage1LandingThrottle.append(stage1Vessel.control.throttle)
                    else:
                        stage1Vessel.control.throttle = LOW
#                        print(stage1Vessel.control.throttle)
                        stage1LandingThrottle.append(stage1Vessel.control.throttle)
                    
                    if stage1Vessel.situation == conn.space_center.VesselSituation.splashed or stage1Vessel.situation == conn.space_center.VesselSituation.landed:
                        stage1landingBurn = False
                        stage1Vessel.control.throttle = 0
    
        #Checking status of stage 1
        if stage1Vessel.situation == conn.space_center.VesselSituation.splashed or stage1Vessel.situation == conn.space_center.VesselSituation.landed:
            stage1landingBurn = False
            stage1Vessel.control.throttle = 0
            if stage1landed == False:
                print('stage 1 landed')
                stage1landed = True
                print('switch to main Vessel')

#                    if stage1Vessel.recoverable == True:
#                        time.sleep(3)
#                        print('recover Stage 1')
#                        time.sleep(3)
#                        stage1Vessel.recover()
                
                time.sleep(13)
                conn.space_center.active_vessel = mainVessel
                stage1Phase = False
                
                # Set up streams for telemetry (again after switch)
                ut = conn.add_stream(getattr, conn.space_center, 'ut')
                altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
                apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
                periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')
                time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
                time_to_periapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_periapsis')
                
                stage2Phase = True
                print('stage2Phase')
    
    elif stage2Phase:
        
        if not stage2Seperation:
            vessel.control.sas = True
            vessel.control.rcs = True
            vessel.control.sas_mode = conn.space_center.SASMode.radial
            time.sleep(10) 
            stage2Shallow = vessel.control.activate_next_stage()
            stage2 = copy.deepcopy(stage2Shallow)
            time.sleep(5)
            print('stage 2 seperated')
            stage2Seperation = True
            
            conn.space_center.active_vessel = stage2[0]
            time.sleep(5)
            
        stage2Vessel = conn.space_center.active_vessel
        ap = stage2Vessel.auto_pilot
            
        #rename Vessel
        if stage2Vessel.name == mainVessel.name + ' Probe':
            stage2Vessel.name = mainVessel.name + ' Stage 2'

        obt_frame = stage2Vessel.orbit.body.non_rotating_reference_frame
        srf_frame = stage2Vessel.orbit.body.reference_frame
        orb_speed = conn.add_stream(getattr, stage2Vessel.flight(obt_frame), 'speed') 
        altitude = conn.add_stream(getattr, stage2Vessel.flight(), 'surface_altitude')
        srf_speed = conn.add_stream(getattr, stage2Vessel.flight(srf_frame), 'speed')
        long = conn.add_stream(getattr, stage2Vessel.flight(obt_frame), 'longitude')
        
        angle = 62
        position = 0
        ksc_loc = (1.301492-angle*np.pi/180)
        while abs(position - ksc_loc) > 0.01:
            position = (long()+180)*np.pi/180
#            print(abs(position - ksc_loc))
            conn.space_center.rails_warp_factor = 4
        conn.space_center.rails_warp_factor = 0
        
        ap.sas = True
        ap.sas_mode = ap.sas_mode.retrograde
        ap.wait()
        
        while (stage2Vessel.orbit.periapsis_altitude > 0):
                stage2Vessel.control.throttle = 1.0
        stage2Vessel.control.throttle = 0.0
        
        while altitude() > 70000:
            conn.space_center.rails_warp_factor = 4
            pass
        conn.space_center.rails_warp_factor = 0
        print('stage 2 reentry')
        time.sleep(0.3)
        print('deploying A.I.R.B.R.A.K.E.S')
        stage2Vessel.control.set_action_group(4, True)
        stage2Vessel.control.rcs = False
        
        ap.sas_mode = ap.sas_mode.retrograde
        ap.wait()
        
        while altitude() > 55000:
            conn.space_center.physics_warp_factor = 4
            pass
        conn.space_center.physics_warp_factor = 0
        
        while altitude() > 50000:
            pass
        print('stage 2 first slow down burn (if necessary)')
        while (orb_speed() > 1500):
                stage2Vessel.control.throttle = 1.0
        stage2Vessel.control.throttle = 0.0
        
        while altitude() > 10000:
            pass
        print('stage 2 second slow down burn (if necessary)')
        while (srf_speed() > 500):
                stage2Vessel.control.throttle = 1.0
        stage2Vessel.control.throttle = 0.0
        
        while altitude() > 2000:
            pass
        print('stage 2 third slow down burn (if necessary)')
        while (srf_speed() > 200):
                stage2Vessel.control.throttle = 1.0
        stage2Vessel.control.throttle = 0.0
        
        stage2Vessel.control.rcs = True
        while altitude() > 1200:
            pass
        print('stage 2 landing burn')

        stage2Vessel.control.rcs = True
        stage2Vessel.control.sas = False
        vessel.auto_pilot.engage()
        print('ap on')
        vessel.auto_pilot.target_pitch = 90
            
        while altitude() > 50:
            if not stage2Vessel.biome == 'Water':
                if not stage2Parachute:
                    try: 
                        for parachutes in conn.space_center.active_vessel.parts.parachutes: #does not allways work.
                             parachutes.deploy()      
                    except Exception:
                        print('Something went horribly wrong. Switching to PSPP (Panicly Spacebar Press Procedure).')
                        for i in range(5): #spaming spacebar to activate parachutes, because parachutes not allways activate at the first try.
                            stage2Vessel.control.activate_next_stage()
                        print('stage 2 additional parachutes deployed (hopefully)')
                    else:  
                        print('stage 2 additional parachute deployed (hopefully)')
                    finally:
                        stage2Parachute = True
                
            if srf_speed() > altitude()/5:
                stage2Vessel.control.throttle = 0.95
            elif srf_speed() > altitude()/10:
                stage2Vessel.control.throttle = 0.1
            elif srf_speed() > altitude()/15:
                stage2Vessel.control.throttle = 0
            elif stage1Vessel.situation == conn.space_center.VesselSituation.splashed or stage1Vessel.situation == conn.space_center.VesselSituation.landed:
                stage2Vessel.control.throttle = 0
                print('stage 2 landed')
                vessel.auto_pilot.disengage()
                stage2Vessel.control.sas = True
                stage2landed = True
                print('switch to main Vessel')
        
        while altitude() > 2:
            if srf_speed() > 5:
                stage2Vessel.control.throttle = 0.5
            else:
                stage2Vessel.control.throttle = 0
                  
        stage2Vessel.control.throttle = 0
        stage2Vessel.control.rcs = False
        
        if stage2landed == False:
            print('stage 2 landed')
            vessel.auto_pilot.disengage()
            stage2Vessel.control.sas = True
            stage2landed = True
            print('switch to main Vessel')
        
        if stage2landed:
            time.sleep(13)
            conn.space_center.active_vessel = mainVessel
            time.sleep(5) 
            
            # Set up streams for telemetry (again after switch)
            ut = conn.add_stream(getattr, conn.space_center, 'ut')
            altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
            apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
            periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')
            time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
            time_to_periapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_periapsis')
            
            ap = vessel.auto_pilot
            
            stage2Phase = False
            orbitPhase = True
            
# =============================================================================
#                         
# =============================================================================
                        
#            sys.exit()
                    
# =============================================================================
#                     
# =============================================================================
                
    elif orbitPhase:
                    
        alpha = angle_between_vektors(vesselPos(), targetBodyPos())
#        print('alpha DEG', round(alpha*180/np.pi, 4), 'RAD', alpha)
        conn.space_center.rails_warp_factor = 4
    
        #so we start at ...° "behinde" the taret and not ...° "in front of" the target. 
        if alpha > 179*np.pi/180 and not vesselBehindTarget:
            conn.space_center.rails_warp_factor = 3
            print('vessel behind target')
            vesselBehindTarget = True
            
        #stop a little bit befor the target and create node 
        if alpha < (targetAngle()+35.1)*np.pi/180 and vesselBehindTarget:
            conn.space_center.rails_warp_factor = 0
            vesselBehindTarget = False
            
#            print('vessel agular speed in RAD/s', vessel_Speed())
#            print('vessel agular speed in DEG/s', vessel_Speed()*180/np.pi)
    
            time_to_node = (alpha - targetAngle()*np.pi/180) / vessel_Speed()
            nodeUt = ut() + time_to_node
            
            # Plan MTO burn
            print('Planning MTO burn')
            mu = vessel.orbit.body.gravitational_parameter
            r1 = vessel.orbit.radius_at(nodeUt)
            r2 = target_body.orbit.radius #m
            a = vessel.orbit.semi_major_axis
            delta_v = np.sqrt(mu/r1) * (np.sqrt(2*r2/(r1+r2)) - 1)
            delta_v += 7 #to get a "non collision" trajectory
            print('needed dv:', delta_v)
            
#            print('DEG to node', (alpha - 115*np.pi/180)*180/np.pi)
#            print('time to node', time_to_node)
            
            node = vessel.control.add_node(nodeUt, prograde = delta_v)
            nodePos = node.position(vessel.orbit.body.non_rotating_reference_frame)
            beta = angle_between_vektors(nodePos, targetBodyPos())
            if not (targetAngle() - 6) < (beta*180/np.pi) and (beta*180/np.pi) < (targetAngle() + 6):
                print('target angle', targetAngle())
                print('beta', beta*180/np.pi) #Kontrolle: sollte etwa dem targetAngle entsprechen.
                print('Something went wrong. Beta does not match with Target Angle.')
                sys.exit()
            
            # Calculate burn time (using rocket equation)
            F = vessel.available_thrust
            Isp = vessel.specific_impulse * vessel.orbit.body.surface_gravity
            m0 = vessel.mass
            m1 = m0 / np.exp(abs(delta_v)/Isp)
            flow_rate = F / Isp
            burn_time = (m0 - m1) / flow_rate
            
            # Wait until burn
            print('Waiting until MTO burn')
            burn_ut = nodeUt - (burn_time/2.) #burn_ut = time to start burn
            lead_time = 10
            conn.space_center.warp_to(burn_ut - lead_time)
            
            # Orientate ship
            vessel.control.sas = True
            vessel.control.sas_mode = conn.space_center.SASMode.maneuver
            
            orbitPhase = False
            mtoPhase = True
        
    elif mtoPhase:
        #Execute burn
        if node.time_to < (burn_ut - lead_time):
            if fineTuneMTO == False:
                print('Ready to execute burn')
                while node.time_to - (burn_time/2.) > 0:
                    pass
                print('Executing burn')
                vessel.control.throttle = 1.0
                while node.remaining_delta_v > (node.delta_v * 0.01):
                    pass
                fineTuneMTO = True
                endBurnMTO = False
                
            elif fineTuneMTO == True and endBurnMTO == False:
                print('Fine tuning MTO')
                vessel.control.throttle = 0.05  
                vessel.control.rcs = False
                try:
                    vessel.control.sas_mode = conn.space_center.SASMode.maneuver
                except Exception as e:
                    print(e)
                    try:
                        vessel.control.sas_mode = conn.space_center.SASMode.prograde
                    except Exception as e:
                        print(e)
                while node.remaining_delta_v >= 0.1:
                    pass
                
                #TECO 1
                vessel.control.throttle = 0.0
                print('TECO 1')
                node.remove()
                conn.space_center.clear_target()
                vessel.control.sas_mode = conn.space_center.SASMode.retrograde
                time.sleep(2)
                endBurnMTO = True
                
                #Wait until Mun SOI
                if not munSoi:
                    conn.space_center.warp_to(ut() + vessel.orbit.time_to_soi_change)
                    time.sleep(3)
                    munSoi = True
                
                # Plan capture burn (using vis-viva equation)
                print('Planning capture burn')
                mu = vessel.orbit.body.gravitational_parameter
                m = vessel.mass
                G = 6.67428e-11
                r = vessel.orbit.periapsis
                a = vessel.orbit.semi_major_axis
                v1 = np.sqrt((mu + G*m) * ((2./r) - (1./a)))
                v2 = np.sqrt((mu + G*m) * (1./r))
                delta_v = v2 - v1
                print('needed dv:', delta_v)
                node = vessel.control.add_node(ut() + vessel.orbit.time_to_periapsis, prograde = delta_v)
                
                # Calculate burn time (using rocket equation)
                F = vessel.available_thrust
                Isp = vessel.specific_impulse * vessel.orbit.body.surface_gravity
                m0 = vessel.mass
                m1 = m0 / np.exp(abs(delta_v)/Isp)
                flow_rate = F / Isp
                burn_time = (m0 - m1) / flow_rate
                
                nodeUt = ut() + node.time_to
                
                # Wait until burn
                print('Waiting until capture burn')
                burn_ut = nodeUt - (burn_time/2.) #burn_ut = time to start burn
                lead_time = 10
                conn.space_center.warp_to(burn_ut - lead_time)
                
                vessel.control.sas_mode = conn.space_center.SASMode.maneuver
                vessel.control.rcs = True
        
                mtoPhase = False
                captureBurnPhase = True
    
    elif captureBurnPhase:
            
        #Execute burn
        if node.time_to < (burn_ut - lead_time):
            if fineTuneCapture == False:
                print('Ready to execute burn')
                                
                while node.time_to - (burn_time/2.) > 0:
                    pass
                print('Executing burn')
                vessel.control.throttle = 1.0
                while node.remaining_delta_v > (node.delta_v * 0.01):
                    pass
                fineTuneCapture = True
                endCaptureBurn = False
                
            elif fineTuneCapture == True and endCaptureBurn == False:
                print('Fine tuning capture burn')
                vessel.control.throttle = 0.05  
                vessel.control.rcs = False
                try:
                    vessel.control.sas_mode = conn.space_center.SASMode.maneuver
                except Exception:
                    print('Cannot set SAS mode of vessel to maneuver')
                    try:
                        vessel.control.sas_mode = conn.space_center.SASMode.prograde
                    except Exception:
                        print('Cannot set SAS mode of vessel to prograde')
                        time.sleep(1)
                        vessel.control.throttle = 0.0
                finally:
                    while node.remaining_delta_v >= 3:
                        pass
                
                    #TECO 2
                    vessel.control.throttle = 0.0
                    print('TECO 2')
                    node.remove()
                    conn.space_center.clear_target()
                    vessel.control.sas_mode = conn.space_center.SASMode.retrograde
                    time.sleep(2)
                    endCaptureBurn = True
                    captureBurnPhase = False

                    #fine tune Orbit
                    if apoapsis() > 150000:
                        
                        conn.space_center.warp_to(ut() + time_to_periapsis() - 10)
                        ap.sas_mode = ap.sas_mode.retrograde
                        ap.wait()
                        
                        while periapsis() > 200000:
                            vessel.control.throttle = 1
                        while periapsis() > 150000:
                            vessel.control.throttle = 0.1
                        vessel.control.throttle = 0
                        
                        conn.space_center.warp_to(ut() + time_to_periapsis() - 10)
                        ap.sas_mode = ap.sas_mode.retrograde
                        ap.wait()
                        
                        while apoapsis() > 200000:
                            vessel.control.throttle = 1
                        while apoapsis() > 150000:
                            vessel.control.throttle = 0.1
                        vessel.control.throttle = 0
                        
# =============================================================================
#                         
# =============================================================================
                        
#                    sys.exit()
                    
# =============================================================================
#                     
# =============================================================================
                                         
                    vessel.control.speed_mode = vessel.control.speed_mode.orbit
                    ap.sas = True
                    ap.sas_mode = ap.sas_mode.retrograde
                    time.sleep(1)
                    
                    munLandingPhase = True
                    
                    print('munLandingPhase')
                    sc.quicksave()
                
    elif munLandingPhase:     
#        print('test')                   
        sun = conn.space_center.bodies['Sun']
        mun = conn.space_center.bodies['Mun']
        kerbin = conn.space_center.bodies['Kerbin']
        
#        print(alphaTested)

        #to land at day time
        if not alphaTested:
            alpha1 = angle_between_vektors(sunPos(), munPos())
            conn.space_center.rails_warp_factor = 5
            time.sleep(1)
            conn.space_center.rails_warp_factor = 0
            alpha2 = angle_between_vektors(sunPos(), munPos())
            
            delta_alpha = alpha1 - alpha2
            
            alphaTested = True
        
        if delta_alpha < 0:
            increasing = True #day side
            decreasing = False #night side
        
        elif delta_alpha > 0:
            increasing = False #day side
            decreasing = True #night side
            
        print('Wait to land at day.')
            
        if increasing:
            alpha = angle_between_vektors(sunPos(), munPos())
            while alpha < 30*np.pi/180:
                conn.space_center.rails_warp_factor = 5
                alpha = angle_between_vektors(sunPos(), munPos())
            conn.space_center.rails_warp_factor = 0
            
            if alpha > 170*np.pi/180:
                while alpha < 179*np.pi/180:
                    conn.space_center.rails_warp_factor = 5
                    alpha = angle_between_vektors(sunPos(), munPos())
                time.sleep(3)
                conn.space_center.rails_warp_factor = 0
                
                while alpha > 35*np.pi/180:
                    conn.space_center.rails_warp_factor = 6
                    alpha = angle_between_vektors(sunPos(), munPos())
                while alpha > 1*np.pi/180:
                    conn.space_center.rails_warp_factor = 5
                    alpha = angle_between_vektors(sunPos(), munPos())
                time.sleep(3)
                conn.space_center.rails_warp_factor = 0
                
                while alpha < 30*np.pi/180:
                    conn.space_center.rails_warp_factor = 5
                    alpha = angle_between_vektors(sunPos(), munPos())
                conn.space_center.rails_warp_factor = 0
        
        elif decreasing:
            alpha = angle_between_vektors(sunPos(), munPos())
            while alpha > 35*np.pi/180: 
                conn.space_center.rails_warp_factor = 6
                alpha = angle_between_vektors(sunPos(), munPos())
            while alpha > 1*np.pi/180:
                conn.space_center.rails_warp_factor = 5
                alpha = angle_between_vektors(sunPos(), munPos())
            time.sleep(3)
            conn.space_center.rails_warp_factor = 0
            
            while alpha < 30*np.pi/180:
                conn.space_center.rails_warp_factor = 5
                alpha = angle_between_vektors(sunPos(), munPos())
            conn.space_center.rails_warp_factor = 0

        canvas = conn.ui.stock_canvas

        panel = canvas.add_panel()
        
        rect = panel.rect_transform
        rect.size = (200,135)
        rect.position = (373,207)
        
        # Settings for text size in the panel on screen
        textAltitude = panel.add_text('Altitude: ')
        textAltitude.rect_transform.position = (0,40)
        textAltitude.color = (1,1,1)
        textAltitude.size = 18
        
        textNext1 = panel.add_text('Wait for deorbiting.')
        textNext1.rect_transform.position = (0,20)
        textNext1.color = (1,1,1)
        textNext1.size = 18
        
        print('Wait for deorbiting.')
        
        textNext2 = panel.add_text('')
        textNext2.rect_transform.position = (0,0)
        textNext2.color = (1,1,1)
        textNext2.size = 18
        
        textNext3 = panel.add_text('')
        textNext3.rect_transform.position = (0,-20)
        textNext3.color = (1,1,1)
        textNext3.size = 18
                    
        vessel.control.set_action_group(3, True)
        time.sleep(1)
        stage3Shallow = vessel.control.activate_next_stage()
        stage3 = copy.deepcopy(stage3Shallow)
        time.sleep(3)
        vessel.control.set_action_group(1, True)
        vessel.control.set_action_group(2, True)
        time.sleep(5)
        
        #Setup for landing
        vessel = conn.space_center.active_vessel

        obt_frame = vessel.orbit.body.non_rotating_reference_frame
        srf_frame = vessel.orbit.body.reference_frame
        
        orb_speed = conn.add_stream(getattr, vessel.flight(obt_frame), 'speed')
        altitude = conn.add_stream(getattr, vessel.flight(), 'surface_altitude')
        srf_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'speed')
        vertical_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'vertical_speed')
        horizontal_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'horizontal_speed')
        time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
        long = conn.add_stream(getattr, vessel.flight(obt_frame), 'longitude')

        if not vessel.name == missionName + ' Lander':
            vessel.name = missionName + ' Lander'
            vessel.type = sc.VesselType.lander

        while long() > -15 or long() < -16:
            textUpdate()
#            textNext3.content = str(long())
            conn.space_center.rails_warp_factor = 5
            pass
        conn.space_center.rails_warp_factor = 0
        print('start deorbiting')
        
        time.sleep(2.5)
        ap.sas = True
        vessel.control.rcs = True
        ap.sas_mode = ap.sas_mode.retrograde
        ap.wait()
        while (vessel.orbit.periapsis_altitude > 0):
            textUpdate()
            vessel.control.throttle = 1.0
        vessel.control.throttle = 0.0
        vessel.control.rcs = False
        
        textNext1.content = 'At 11000 m:'
        textNext2.content = 'kill horizontal speed'
        textNext3.content = ''
        print('kill horizontal speed')

        while altitude() > 80000:
            textUpdate()
            conn.space_center.rails_warp_factor = 4
        while altitude() > 30000:
            textUpdate()
            conn.space_center.rails_warp_factor = 3
        while altitude() > 15000:
            textUpdate()
            conn.space_center.rails_warp_factor = 2
        while altitude() > 11000:
            textUpdate()
            conn.space_center.rails_warp_factor = 1
        conn.space_center.rails_warp_factor = 0
        
        ap.sas = True
        ap.sas_mode = ap.sas_mode.retrograde  
        ap.wait()
                
        while horizontal_speed() > 20:
            textUpdate()
            vessel.control.throttle = 1
        vessel.control.throttle = 0
                
        h = suicideBurn()
        textUpdate()
        SBA = altitude() - (h + 50)
     
        burnCheck = True
        
        #bugfixing: check for unrealistic h values
        while h > 100 and altitude() > 1000:
            print('unrealistic h value')
            print(h)
            v = vertical_speed()
            g = vessel.orbit.body.surface_gravity
            m = vessel.mass
            F = vessel.available_thrust
            a = F/m - g
            h = v ** 2 / (2*a)
                 
            print('v', v)
            print('g', g)
            print('m', m)
            print('F', F)
            print('a', a)
            print('h', h)
            print('alt', altitude())
            
            textUpdate()
            burnCheck = True
            h = suicideBurn()
            if altitude() < 1050:
                burnCheck = False

#        print('h', h)
#        print('alt', altitude())
#        print('SBA', SBA)

        while burnCheck:
            h = suicideBurn()
            textUpdate()
            SBA = altitude() - (h + 50)
            if SBA > 0:
                textNext1.content = 'Suicide burn in:'
                textNext2.content = str(round(SBA, 1)) + 'm'
            else:
                textNext1.content = 'Suicide burn'
                textNext2.content = ''
                print('Suicide burn')
#                print('SBA', SBA)
            if (h + 50) > altitude() and altitude() < 50000:
                textNext2.content = ''
                burnCheck = False
            pass
        
        while orb_speed() > 10:
            textUpdate()
            vessel.control.throttle = 1.0
        vessel.control.throttle = 0.0
        
        vessel.control.rcs = True
        while altitude() > 1:
            textUpdate()
            if srf_speed() > 2:
                vessel.control.throttle = 0.3
            else:
                vessel.control.throttle = 0
            if vessel.situation == conn.space_center.VesselSituation.landed:
                break

        vessel.control.throttle = 0
        print('ready for touchdown')
                         
        while not vessel.situation == conn.space_center.VesselSituation.landed:
            pass
        
        vessel.control.rcs = False
        print('Touchdown!')
        time.sleep(1)
        print('The Falcon has landed!')
        time.sleep(3)
        print('Doing science!')
        time.sleep(1)
        print('Done in 5...')
        time.sleep(1)
        print('4...')
        time.sleep(1)
        print('3...')
        time.sleep(1)
        print('2...')
        time.sleep(1)
        print('1...')
        time.sleep(1)
        print('done')
        time.sleep(1)
        print('Waiting for time to orbit.')
        
        textAltitude.remove()
        panel.remove()
    
        mun_turn_start_altitude = 150
        mun_turn_end_altitude = 25000 #decides how agressive the turn is. The lower, the more aressive. 
        target_mun_altitude = 100000
        
        conn.space_center.target_vessel = stage3[0]
        
        munLandingPhase = False
        munLiftOffPhase = True
        
        print('munLiftOffPhase')
        
    elif munLiftOffPhase:
        alpha = angle_between_vektors(vesselPos(), targetPos())
#        print('alpha DEG', round(alpha*180/np.pi, 4), 'RAD', alpha, munVesselBehindTarget)
        conn.space_center.rails_warp_factor = 5
    
        #so we start at ...° "behinde" the taret and not ...° "in front of" the target.. 
        if alpha < (1)*np.pi/180 and not munVesselBehindTarget:
            conn.space_center.rails_warp_factor = 3
            print('vessel behind target')
            munVesselBehindTarget = True
            
        #stop a little bit befor the target and create node 
        if alpha > (20)*np.pi/180 and munVesselBehindTarget:
            print('ready for Mun Lift off')
            conn.space_center.rails_warp_factor = 0
            munVesselBehindTarget = False
            
            altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
            time.sleep(3)
            vessel.control.activate_next_stage()
            time.sleep(0.1)
            vessel.control.activate_next_stage() #10
            time.sleep(0.3)            
            
            ap.sas = True
            ap.engage()
            vessel.control.rcs = False
            vessel.control.throttle = 1
            print('munAccentPhase')

            turn_angle = 0
        
            munAccentPhase = True
            munLiftOffPhase = False
        
    elif munAccentPhase:
        # Gravity turn
        if not munGravityTurn:
            if altitude() > mun_turn_start_altitude and altitude() < mun_turn_end_altitude:
                frac = ((altitude() - mun_turn_start_altitude) /
                        (mun_turn_end_altitude - mun_turn_start_altitude))
                new_turn_angle = frac * 90
                if abs(new_turn_angle - turn_angle) > 0.5:
                    turn_angle = new_turn_angle
                    vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 270)
        
            # Decrease throttle when approaching target apoapsis
            if apoapsis() > target_mun_altitude * 0.9:
                print('Approaching target apoapsis')
                
                # Disable engines when target apoapsis is reached
                vessel.control.throttle = 0.25
                while apoapsis() < target_mun_altitude:
                    pass
                munGravityTurn = True
                vessel.auto_pilot.disengage()
    
        #SLECO 1
        if apoapsis() > target_mun_altitude:
            vessel.control.throttle = 0
            time.sleep(0.5)
            print('SLECO 1')
            
            vessel.control.sas = True
            vessel.control.rcs = True
            time.sleep(0.1)
            vessel.control.sas_mode = conn.space_center.SASMode.prograde
            time.sleep(3)
    
            munAccentPhase = False
            munCruisePhase = True
            print('munCruisePhase')
            
            # Plan circularization burn (using vis-viva equation)
            print('Planning circularization burn')
            mu = vessel.orbit.body.gravitational_parameter
            m = vessel.mass
            G = 6.67428e-11
            r = vessel.orbit.apoapsis
            a = vessel.orbit.semi_major_axis
            v1 = np.sqrt((mu + G*m) * ((2./r) - (1./a)))
            v2 = np.sqrt((mu + G*m) * (1./r))
            delta_v = v2 - v1
            print('needed dv:', delta_v)
            node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)
            
            # Calculate burn time (using rocket equation)
            F = vessel.available_thrust
            Isp = vessel.specific_impulse * 9.82
            m0 = vessel.mass
            m1 = m0 / np.exp(delta_v/Isp)
            flow_rate = F / Isp
            burn_time = (m0 - m1) / flow_rate
            
            # Wait until burn
            print('Waiting until circularization burn')
            burn_ut = ut() + time_to_apoapsis() - (burn_time/2.)
            lead_time = 6
            conn.space_center.warp_to(burn_ut - lead_time)
            munCruisePhase = False
            munInsertionPhase = True
            print("munInsertionPhase")
            
            # Orientate ship
            vessel.control.sas_mode = conn.space_center.SASMode.maneuver
                
    elif munInsertionPhase:

        # Execute burn
        if time_to_apoapsis() < (burn_ut - lead_time):
            if munFineTune1 == False:
                print('Ready to execute burn')
                time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
                while time_to_apoapsis() - (burn_time/2.) > 0:
                    pass
                print('Executing burn')
                vessel.control.throttle = 1.0
                while node.remaining_delta_v > (node.delta_v * 0.01):
                    pass
                munFineTune1 = True
                munEndBurn1 = False
                
            elif munFineTune1 == True and munEndBurn1 == False:
                print('Fine tuning')
                vessel.control.throttle = 0.05
                vessel.control.rcs = False
                try:
                    vessel.control.sas_mode = conn.space_center.SASMode.maneuver
                except Exception as e:
                    print(e)
                    try:
                        vessel.control.sas_mode = conn.space_center.SASMode.prograde
                    except Exception as e:
                        print(e)
                while node.remaining_delta_v >= 0.1:
                    pass
                
                #SLECO 2
                vessel.control.throttle = 0.0
                print('SLECO 2')
                node.remove()
                munEndBurn1 = True

                landerStage2Shallow = conn.space_center.active_vessel
                landerStage2 = copy.deepcopy(landerStage2Shallow)
                print('Switching to stage 3 for redevouz.')
                
                time.sleep(3)
                conn.space_center.active_vessel = stage3[0]
                time.sleep(3)
                
                munInsertionPhase = False
                munRendevouzPhase = True
                
                print('munRendevouzPhase')
                
                
    elif munRendevouzPhase:
        
        conn.space_center.target_vessel = landerStage2
        target_vessel = conn.space_center.target_vessel
        vessel = conn.space_center.active_vessel
        
        time.sleep(2)
    
        #This script assumes the vessel is in orbit, planes match and the target is set.
        print("Planning Hohmann transfer")
        planner = mj.maneuver_planner
        hohmann = planner.operation_transfer
        makeHohmann = True
        while makeHohmann:
            try: 
                node = hohmann.make_node()
            except Exception as warning:
                try:
                    node.remove()
                except:
                    pass
                print(warning)
                makeHohmann = True
                print('Wait one orbit and try again.')
                conn.space_center.warp_to(ut() + vessel.orbit.period)
            else:
                makeHohmann = False
            pass
                 
        #execute the node
        executor = mj.node_executor
        execute_node()
         
        #fine tune closest approach to the target
        print("Correcting course")
        fineTuneClosestApproach = planner.operation_course_correction
        fineTuneClosestApproach.intercept_distance = 50 #50 meters seems to be optimal distance; if you use 0, you can hit the target
        fineTuneClosestApproach.make_node()
        executor.tolerance = 0.01 #do a high-precision maneuver (0.01 dV tolerance)
        execute_node()
        
        time.sleep(1)
        
        #Matching speed with the target
        print("Matching speed with the target")
        matchSpeed = planner.operation_kill_rel_vel
        matchSpeed.time_selector.time_reference = mj.TimeReference.closest_approach #match speed at the closest approach
        node = matchSpeed.make_node()
        executor.tolerance = 0.1 #return the precision back to normal
        execute_node()
        
        time.sleep(3)
        
        print("Rendezvous complete!")
        
        munRendevouzPhase = False
        munDockingPhase = True
        print('munDockingPhase')

    elif munDockingPhase:
        
        vessel.control.sas = True
        vessel.control.rcs = True
        
        lastDistance = targetDistance()
        print('Wait until closest approach')
        time.sleep(0.3)
        
        try:
            lastDistance = targetDistance()
            time.sleep(0.3)
            
            while lastDistance > targetDistance():
                if targetDistance() < 20:
                    break
                if targetDistance() > 50:
                    conn.space_center.rails_warp_factor = 3
                elif targetDistance() > 30:
                    conn.space_center.rails_warp_factor = 2
                else:
                    conn.space_center.rails_warp_factor = 1     
                lastDistance = targetDistance()
                time.sleep(0.1)
                pass
            conn.space_center.rails_warp_factor = 0
        except Exception:
            pass
        
        print('Switch to second lander stage')
        
        time.sleep(2)
        sc.active_vessel = landerStage2
        time.sleep(3)
        
        conn.space_center.target_vessel = stage3[0]
        target_vessel = conn.space_center.target_vessel
        vessel = conn.space_center.active_vessel
        
        time.sleep(1)
        
        #This script assumes the vessel is next to the target and the target is a ship.
        active = sc.active_vessel
        
        print("Setting the first docking port as the controlling part")
        parts = active.parts
        parts.controlling = parts.docking_ports[0].part
        
        print("Looking for a free docking port attached to the target vessel")
        for dp in sc.target_vessel.parts.docking_ports:
            if not dp.docked_part:
                sc.target_docking_port = dp
                break    
        
        print("Starting the docking process")
        docking = mj.docking_autopilot
        docking.enabled = True
        
        with conn.stream(getattr, docking, "enabled") as enabled:
            enabled.rate = 1 #we don't need a high throughput rate, 1 second is more than enough
            with enabled.condition:
                while enabled():
                    enabled.wait()
        
        print("Docking complete!")
#        panel.remove()    
        time.sleep(1)
        
        vessel = conn.space_center.active_vessel

        try:
            if not vessel.name == missionName + ' Ship':
                vessel.name = missionName + ' Ship'
                vessel.type = sc.VesselType.ship
        except:
            pass
            
        conn.space_center.active_vessel.parts.engines[0].active = False
        
        time.sleep(2)
        
        munDockingPhase = False
        munOrbitPhase = True 
        print('munOrbitPhase')
    
    elif munOrbitPhase:
        kerbin = conn.space_center.bodies['Kerbin']
        
        alpha = angle_between_vektors(vesselPos(), kerbinPos())
#        print('alpha DEG', round(alpha*180/np.pi, 4), 'RAD', alpha)
        conn.space_center.rails_warp_factor = 5
    
        #so we start at ...° "behinde" the taret and not ...° "in front of" the target. 
        if alpha < 5*np.pi/180 and not munVesselBehindTarget:
            conn.space_center.rails_warp_factor = 0
            print('vessel behind target')
            munVesselBehindTarget = True
                        
#            print('vessel agular speed in RAD/s', vessel_Speed())
#            print('vessel agular speed in DEG/s', vessel_Speed()*180/np.pi)
    
            time_to_node = (180*np.pi/180 - alpha) / vessel_Speed()
            nodeUt = ut() + time_to_node
            
            # Plan KTO burn
            print('Planning KTO burn')
            mu = kerbin.gravitational_parameter
            r1 = 1000
            r2 = vessel.orbit.body.orbit.radius #m
            a = vessel.orbit.semi_major_axis
            delta_v = np.sqrt(mu/r2) * (1 - (np.sqrt(2*r1/(r2+r1))))
            print('needed dv:', delta_v)
            
#            print('DEG to node', (alpha - 115*np.pi/180)*180/np.pi)
#            print('time to node', time_to_node)
            
            node = vessel.control.add_node(nodeUt, prograde = delta_v)
            
            # Calculate burn time (using rocket equation)
            F = vessel.available_thrust
            Isp = vessel.specific_impulse * vessel.orbit.body.surface_gravity
            m0 = vessel.mass
            m1 = m0 / np.exp(abs(delta_v)/Isp)
            flow_rate = F / Isp
            burn_time = (m0 - m1) / flow_rate
            
            # Wait until burn
            print('Waiting until KTO burn')
            burn_ut = nodeUt - (burn_time/2.) #burn_ut = time to start burn
            lead_time = 10
#            print('nodeUt', nodeUt)
#            print('burn_ut', burn_ut)
#            print('ut', ut())
            conn.space_center.warp_to(burn_ut - lead_time)
        
            # Orientate ship
            ap.disengage()
            time.sleep(0.5)
            vessel.control.set_action_group(5, True)
            time.sleep(0.5)
            vessel.control.sas = True
            time.sleep(0.5)
            vessel.control.sas_mode = conn.space_center.SASMode.maneuver
            
            munOrbitPhase = False
            ktoPhase = True
            print('ktoPhase')
        
    elif ktoPhase:
        #Execute burn
        if node.time_to < (burn_ut - lead_time):
            if fineTuneKTO == False:
                print('Ready to execute burn')
                while node.time_to - (burn_time/2.) > 0:
                    pass
                print('Executing burn')
                vessel.control.throttle = 1.0
                while node.remaining_delta_v > (node.delta_v * 0.01):
                    pass
                fineTuneKTO = True
                endBurnKTO = False
                
            elif fineTuneKTO == True and endBurnKTO == False:
                print('Fine tuning KTO')
                vessel.control.throttle = 0.05  
                vessel.control.rcs = False
                try:
                    vessel.control.sas_mode = conn.space_center.SASMode.maneuver
                except Exception as e:
                    print(e)
                    try:
                        vessel.control.sas_mode = conn.space_center.SASMode.prograde
                    except Exception as e:
                        print(e)
                while node.remaining_delta_v >= 0.1:
                    pass
                
                #TECO 2
                vessel.control.throttle = 0.0
                print('TECO 2')
                node.remove()
                time.sleep(2)
                endBurnKTO = True
                
                #Wait until Kerbin SOI
                if not kerbinSoi:
                    conn.space_center.warp_to(ut() + vessel.orbit.time_to_soi_change)
                    time.sleep(3)
                    kerbinSoi = True
                
                ktoPhase = False
                mainReentryPhase = True
                print('mainReentryPhase')
                
    elif mainReentryPhase:
        
        altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
          
        try:
            ap.sas_mode = ap.sas_mode.retrograde
        except:
            pass
        
#        stageInWhichStage3Decouple = 1
#        stage_3_decouple_resources = vessel.resources_in_decouple_stage(stage=stageInWhichStage3Decouple, cumulative=False)
#        liquid_fuel = conn.add_stream(stage_3_decouple_resources.amount, 'LiquidFuel')
        
        if not mainReentryPeriapsis:
            print('Warp to periapsis')
            conn.space_center.warp_to(ut() + vessel.orbit.time_to_periapsis)
            mainReentryPeriapsis = True
        
            print('deorbiting')
#            while liquid_fuel() > 0:
#                vessel.control.throttle = 1
#            vessel.control.throttle = 0
            
            while conn.space_center.active_vessel.parts.engines[1].has_fuel:
                vessel.control.throttle = 1
            vessel.control.throttle = 0
    
    
            ap.engage()
            altitude = conn.add_stream(getattr, vessel.flight(), 'surface_altitude')
            vessel.control.set_action_group(5, False)
            vessel.control.set_action_group(6, True)
            ap.sas_mode = ap.sas_mode.anti_radial
            ap.wait()
            print('reentry modul seperated')
            vessel.control.activate_next_stage()
            vessel.control.activate_next_stage()
            time.sleep(0.5)
            ap.disengage()
            time.sleep(0.5)
            ap.sas = True
            time.sleep(0.5)
            ap.sas_mode = ap.sas_mode.prograde
            ap.wait()
            MRE_t = 0
            
            while altitude() > 100000:
                conn.space_center.rails_warp_factor = 2
                pass
            while altitude() > 75000:
                conn.space_center.rails_warp_factor = 1
                pass
            conn.space_center.rails_warp_factor = 0
            
            while altitude() > 70000:
                mainReentryData()
                MRE_t += 1
                pass
            print('start reentry')
            
            while altitude() > 10000:
                mainReentryData()
                MRE_t += 1
                pass
            
            try:
                for parachutes in conn.space_center.active_vessel.parts.parachutes: #does not allways work.
                     parachutes.deploy()      
            except Exception as e:
                print('!!!!!!!!!!!!!!!!!!!!!!!!!!')
                print('')
                print(e)
                print('')
                print('Something went horribly wrong. Switching to PSPP (Panicly Spacebar Press Procedure).')
                print('')
                print('!!!!!!!!!!!!!!!!!!!!!!!!!!')
                for i in range(5): #spaming spacebar to activate parachutes, because parachutes not allways activate at the first try.
                    vessel.control.activate_next_stage()
                print('parachutes deployed (hopefully)')
            else:  
                print('parachutes deployed (hopefully)')  
            finally:
                vessel.control.sas = False
            
        mainReentryData()
        MRE_t += 1

        if vessel.situation == conn.space_center.VesselSituation.splashed or vessel.situation == conn.space_center.VesselSituation.landed:
            print('landed')
            vessel.control.sas = False
            mainReentryPhase = False

#stage 1
plt.figure(1)
plt.subplot(211)
plt.plot(stage1_t_list, stage1_altitude_list, label='Stage 1 altitude')
plt.legend(loc='best', frameon=False)
plt.grid

plt.subplot(212)
plt.plot(stage1_t_list, stage1_gForce_list, label='Stage 1 $g$')
plt.legend(loc='best', frameon=False)
plt.grid

plt.figure(2)
plt.subplot(311)
plt.plot(stage1LandingTime, stage1LandingAltitude, label='stage1LandingSpeed')
plt.legend(loc='best', frameon=False)
plt.grid

plt.subplot(312)
plt.plot(stage1LandingTime, stage1LandingThrottle, label='stage1LandingThrottle')
plt.legend(loc='best', frameon=False)
plt.grid

plt.subplot(313)
plt.plot(stage1LandingTime, stage1LandingSpeed, label='stage1LandingAltitude')
plt.legend(loc='best', frameon=False)
plt.grid

#main reentry
plt.figure(3)
plt.subplot(311)
plt.plot(MRE_time_list, MRE_altitude_list, label='MRE altitude')
plt.legend(loc='best', frameon=False)
plt.grid

plt.subplot(312)
plt.plot(MRE_time_list, MRE_speed_list, label='MRE speed')
plt.legend(loc='best', frameon=False)
plt.grid

plt.subplot(313)
plt.plot(MRE_time_list, MRE_gForce_list, label='MRE gForce')
plt.legend(loc='best', frameon=False)
plt.grid

plt.show