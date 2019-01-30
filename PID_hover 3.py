import krpc
import time
import matplotlib.pyplot as plt

conn = krpc.connect()
vessel = conn.space_center.active_vessel
ap = vessel.auto_pilot
sc = conn.space_center

def velocity(velocityT):
#    print(round(- srf_velocity()[0], 5), round(srf_velocity()[1], 5), round(srf_velocity()[2], 5))
    velocityXList.append(round(- srf_velocity()[0], 5))
    velocityYList.append(round(srf_velocity()[1], 5))
    velocityZList.append(round(srf_velocity()[2], 5))
    velocityTList.append(velocityT)
    velocityT += 1

    
    surfaceAltitudeList.append(surface_altitude())
    return velocityT
    
def latitude_control():       
    if lat() < -0.0974:
        vessel.control.right = 1
    if -0.0961 < lat(): 
        vessel.control.right = -1
        
    if -0.0974 < lat() and lat() < -0.0961:
        vessel.control.right = 0
        
        if srf_velocity()[1] < -1:
            vessel.control.right = 1
        if srf_velocity()[1] > 1:
            vessel.control.right = -1
            
def PID(lastP, lastTime, totalP):
    output = 0
    now = time.time()
    
    P = 190 - altitude()
    I = 0
    D = 0

    if lastTime > 0:
        I = totalP + ((P + lastP)/2 * (now - lastTime))
        D = (P - lastP) / (now - lastTime)
      
    output = P * kP + I * kI + D * kD
    
    lastP = P
    lastTime = now
    totalP = I
    
    autoThrottle = output
    autoThrottle = max(0, min(autoThrottle, 1))
    vessel.control.throttle = autoThrottle   
    
    #LOG
    hoverTimeList.append(ut() - startTime)
    altitudeList.append(altitude())
    autoThrottelList.append(autoThrottle)
    
    #Textupdate    
    textP.update('up:' + str(vessel.control.up))
    textI.update('forward:' + str(vessel.control.forward))
    textD.update('right:' + str(vessel.control.right))
    textO.update('long:' + str(long()))
    textA.update('lat:' + str(lat()))
    textT.update('Time:' + str(round((ut() - startTime), 2)))

    return lastP, lastTime, totalP

class text:
    def __init__(self, Name, Y_Pos):
        self.Name = Name
        self.Y_Pos = Y_Pos
        
        self.text = panel.add_text(Name) 
        self.text.rect_transform.position = (0, Y_Pos)
        self.text.color = (1,1,1)
        self.text.size = 18
    
    def update(self, update):
        self.text.content = update
        
if __name__ == '__main__':
    
    srf_frame = vessel.orbit.body.reference_frame
    obt_frame = vessel.orbit.body.non_rotating_reference_frame

    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    surface_altitude = conn.add_stream(getattr, vessel.flight(), 'surface_altitude')
    horizontal_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'horizontal_speed')
    vertical_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'vertical_speed')
    long = conn.add_stream(getattr, vessel.flight(), 'longitude')
    lat = conn.add_stream(getattr, vessel.flight(), 'latitude')
    fuel = conn.add_stream(vessel.resources.amount, 'LiquidFuel')
    srf_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'speed') 
    srf_velocity = conn.add_stream(getattr, vessel.flight(srf_frame), 'velocity')
    
    time.sleep(0.001)
    
    #Set up variables        
    kP = 0.01
    kI = 0.0005
    kD = 0.005
    
    hovering = False
    hoveringEnd = False
    Phase1 = True
    Phase2 = False
    
    autoThrottle = 0
    lastP = 0
    lastTime = 0
    totalP = 0
    lastAltitude = 0
    velocityT = 0 
    
    velocityXList = []
    velocityYList = []
    velocityZList = []
    velocityTList = []
    
    altitudeList = []
    hoverTimeList = []
    autoThrottelList = []
    surfaceAltitudeList = []
    
    #Set up text
    canvas = conn.ui.stock_canvas
    
    panel = canvas.add_panel()
    
    rect = panel.rect_transform
    rect.size = (200,135)
    rect.position = (373,207)
    
    # Settings for text size in the panel on screen
    textP = text('up: ' + str(vessel.control.up), 45)
    textI = text('forward: ' + str(vessel.control.forward), 25)
    textD = text('right: ' + str(vessel.control.right), 5)
    textO = text('long: ' + str(long()), -15)
    textA = text('lat: ' + str(lat()), -35)
    textT = text('Time: 0', -55)
    
    #pre flight check
    if fuel() > 0:
        ap.engage()
        ap.target_pitch_and_heading(90, 270)
        print('Autopilot online')
        time.sleep(0.3)
        vessel.control.throttle = 0.5
        print('Throttle 50%')
        time.sleep(0.3)
        print("Pre flight checks done.")
        time.sleep(0.3)
        print('Go for Lift Off.')
        time.sleep(0.3)
        vessel.control.activate_next_stage()
        print('Lift Off')
        ap.target_pitch_and_heading(90, 270)
        ap.wait()
        
        #Get us 200 meters up
        while altitude() < 190:
            pass
        startTime = ut()
        hovering = True
        vessel.control.rcs = True
        print('start hovering')
        
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
        print('No fuel')
    
    while hovering:
        velocityT = velocity(velocityT)
                
        #hovering
        lastP, lastTime, totalP = PID(lastP, lastTime, totalP)
        
        time.sleep(0.003)

        #Check if no fuel left
        if fuel() == 0 and hovering:
            print('No fuel left.')
            print('End hovering.')
            vessel.control.throttle = 0
            vessel.control.activate_next_stage()
            panel.remove()
            hovering = False
                  
        #wait until 'good' hovering
        if (ut() - startTime) > 60 and Phase1:
            vessel.control.up = -1
            if srf_velocity()[0] < -10:
                Phase1 = False 
                Phase2 = True
                
        if Phase2:
            velocityT = velocity(velocityT)
            
            latitude_control()
                
            #travel with constant speed
            if  srf_velocity()[0] < -10:
                vessel.control.up = 1
            else: 
                vessel.control.up = -1   
                
            #slow down   
            if long() < -74.61:
                vessel.control.up = 0.2
                if srf_velocity()[0] > -1:
                    vessel.control.up = 0  
                    
            #land
            if long() < -74.615:
                vessel.control.up = 0
                print('landing')
                hovering = False
                landing = True
                
                while landing:
                    velocityT = velocity(velocityT)
                    
                    #slow down
                    if abs(vertical_speed()) > 1:
                        vessel.control.throttle = 0.3
                    else:
                        vessel.control.throttle = 0
                    
                    #landing burn
                    if srf_velocity()[0] < -1:
                        vessel.control.up = 0.5
                    else: 
                        vessel.control.up = -0.5
                        
                    latitude_control()
                                            
                    if vessel.situation == conn.space_center.VesselSituation.landed:
                        vessel.control.throttle = 0
                        landing = False
                        vessel.control.rcs = False
                        panel.remove()
                        vessel.control.set_action_group(1, True)
                        time.sleep(10)

    plt.figure(1)
    plt.subplot(211)
    plt.plot(hoverTimeList, altitudeList, label='Altitude')
    plt.legend(loc='best', frameon=False)
    plt.grid()
    
    plt.subplot(212)
    plt.plot(hoverTimeList, autoThrottelList, label='autoThrottel')
    plt.legend(loc='best', frameon=False)
    plt.grid()

# =============================================================================

    plt.figure(2)
    plt.subplot(311)
    plt.plot(velocityTList, velocityXList, label='Velocity X (forward)')
    plt.legend(loc='best', frameon=False)
    plt.grid()
    
    plt.subplot(312)
    plt.plot(velocityTList, velocityYList, label='Velocity Y (right)')
    plt.legend(loc='best', frameon=False)
    plt.grid()
    
    plt.subplot(313)
    plt.plot(velocityTList, velocityZList, label='Velocity Z (up)')
    plt.legend(loc='best', frameon=False)
    plt.grid()
    
# =============================================================================
    
    plt.figure(3)
    plt.plot(velocityTList, surfaceAltitudeList, label='surface Altitude')
    plt.legend(loc='best', frameon=False)
    plt.grid()
        
    plt.show()