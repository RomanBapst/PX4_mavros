#!/usr/bin/env python
"""
This file contains a rospy node which can be used to simulate
a vehicle in JSBSim. The node runs JSBSim as a child process and publishes
the simulation result on the topic "simout". Furthermore, this node accepts
inputs for the simulator on the topic "mixer_out". This file was written to
simulate the Quadshot but it can be adapted to control any vehicle.
Right now, all the settings (for e.g. JSBSim ) are hard coded, this should be
changed in the future to allow convenient handling for other people.

Quick Overview on how this node works:

1) input message (elevons, throttles) arrives on topic "mixer_out"
2) mixer_out callback is exectuted
3) inputs are sent to JSBSim via TCP socket
4) wait for JSBSim to output data via UDP socket
5) send this data to whatever controller you have via UDP on topic "simout"
6) wait until a new message arrives on topic "mixer_out" and execute 1)

Notice that at the moment for the node to work, the user has to inject one
message at the very beginning into the topic mixer_out so that the flow can
start. This should be changed in future.

Many ideas are taken from APM SITL!

Author: Roman Bapst <bapstr@ethz.ch>
Date:   19.09.2014
"""
import sys, os, pexpect, socket, time, signal

from pymavlink import fgFDM
from pysim import util, fdpexpect

import rospy
from simulation.msg import mixer_out
from simulation.msg import simout


fdm = 0
jsb_in = 0
pub = 0
jsb_console = 0
fg_out = 0
jsb = 0
time_now = 0
last_time = 0

class control_state(object):
    def __init__(self):
        self.aileron = 0
        self.elevator = 0
        self.throttle = [0,0,0,0]   #4 engines

sim_state = control_state()

def jsb_set(variable, value):
    '''set a JSBSim variable'''
    global jsb_console
    jsb_console.send('set %s %s\r\n' % (variable, value))

def process_jsb_input(mixer_out_msg):
    global sim_state
    """send ouput from mixer to JSBsim for simulation """
    if mixer_out_msg.aileron != sim_state.aileron:
         jsb_set('fcs/aileron-cmd-norm', mixer_out_msg.aileron)
         sim_state.aileron = mixer_out_msg.aileron
    if mixer_out_msg.elevator != sim_state.elevator:
         jsb_set('fcs/elevator-cmd-norm', -mixer_out_msg.elevator)
         sim_state.elevator = mixer_out_msg.elevator
         
         
    if mixer_out_msg.throttle_0 != sim_state.throttle[0]:
        jsb_set('fcs/throttle-cmd-norm[0]',mixer_out_msg.throttle_0)
        sim_state.throttle[0] = mixer_out_msg.throttle_0
    if mixer_out_msg.throttle_1 != sim_state.throttle[1]:
        jsb_set('fcs/throttle-cmd-norm[1]',mixer_out_msg.throttle_1)
        sim_state.throttle[1] = mixer_out_msg.throttle_1
    if mixer_out_msg.throttle_2 != sim_state.throttle[2]:
        jsb_set('fcs/throttle-cmd-norm[2]',mixer_out_msg.throttle_2)
        sim_state.throttle[2] = mixer_out_msg.throttle_2
    if mixer_out_msg.throttle_3 != sim_state.throttle[3]:
        jsb_set('fcs/throttle-cmd-norm[3]',mixer_out_msg.throttle_3)
        sim_state.throttle[3] = mixer_out_msg.throttle_3

         
def publish(simout_data):
    global fdm, pub, time_now, last_time
    fdm.parse(simout_data)
    try:
        fg_out.send(fdm.pack())
    except:
        pass
    msg = simout()
    msg.roll = fdm.get('phi',units='radians')               #roll
    print(msg.roll)
    msg.pitch = fdm.get('theta',units='radians')            #pitch
    msg.yaw = fdm.get('psi',units='radians')                #yaw
    msg.rollspeed = fdm.get('phidot',units='rps')           #rollspeed
    msg.pitchspeed = fdm.get('thetadot',units='rps')        #pitchspeed
    msg.yawspeed = fdm.get('psidot',units='rps')            #yawspeed
    time_now = rospy.get_time()
    #print("delta_t: ", time_now - last_time)
    last_time = time_now
    
    pub.publish(msg)
    """publish on topic"""

def mixer_out_cb(mixer_out_msg):
    global jsb_in, fdm, jsb_console
    """send data from mixer to JSBsim"""
    process_jsb_input(mixer_out_msg) #inputs are sent to jsbsim
    """"now wait until we get data from jsbsim so that we can publish it
    on a topic"""
   
    simout_data = jsb_in.recv(fdm.packet_size())  #this call blocks until we receive data
    
    publish(simout_data)
    


def run_node():
    global pub
    global fdm
    global jsb_in
    global jsb_console
    global fg_out
    rospy.init_node('simulation', anonymous=True) 
    
    rospy.Subscriber("mixer_out", mixer_out, mixer_out_cb) 
    
    pub = rospy.Publisher("simout",simout,queue_size=1)
    """Start JSBsim and treat it as a child proccess""" 
    cmd = "JSBSim --realtime --suspend --nice --simulation-rate=400 --logdirectivefile=jsbsim/fgout.xml  --script=jsbsim/rascal_test.xml"
    jsb = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
    jsb.delaybeforesend = 0
    util.pexpect_autoclose(jsb)
    i = jsb.expect(["Successfully bound to socket for input on port (\d+)",
                "Could not bind to socket for input"])
    if i == 1:
        print("Failed to start JSBSim - is another copy running?")
        sys.exit(1)
    jsb_out_address = ('127.0.0.1',5002)    #port on which jsbsim takes data
    jsb.expect("Creating UDP socket on port (\d+)")
    jsb_in_address = ('127.0.0.1',5505)     #port on which jsbsim outputs data
    jsb.expect("Successfully connected to socket for output")
    jsb.expect("JSBSim Execution beginning")
    
    # setup output to jsbsim
    print("JSBSim console on %s" % str(jsb_in_address))
    jsb_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    jsb_out.connect(jsb_in_address)
    jsb_console = fdpexpect.fdspawn(jsb_out.fileno(), logfile=sys.stdout)
    jsb_console.delaybeforesend = 0
    
    # setup input from jsbsim
    print("JSBSim FG FDM input on %s" % str(jsb_out_address))
    jsb_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    jsb_in.bind(jsb_out_address)
    #jsb_in.setblocking(0)  #we want this to be blocking
    
    #setput input to flightgear for visualization
    fg_address = ('127.0.0.1',5503)
    fg_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fg_out.connect(fg_address)

    fdm = fgFDM.fgFDM()
    
    jsb_console.send('info\n')
    jsb_console.send('resume\n')
    jsb.expect(["trim computation time","Trim Results"])
    time.sleep(1.5)
    jsb_console.logfile = None
    print("Simulator ready to fly")
    
    rospy.spin()
    
def exit_handler():
    '''exit the sim'''
    global jsb
    global jsb_console
    
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    # JSBSim really doesn't like to die ...
    if getattr(jsb, 'pid', None) is not None:
        os.kill(jsb.pid, signal.SIGKILL)
    jsb_console.send('quit\n')
    util.pexpect_close_all()
    sys.exit(1)
    
signal.signal(signal.SIGINT, exit_handler)
signal.signal(signal.SIGTERM, exit_handler)



if __name__ == '__main__':
    try:
        run_node()
    except:
        exit_handler()
        pass
    
    