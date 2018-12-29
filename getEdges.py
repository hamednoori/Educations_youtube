##### Author: Mahboobe Rezaee #####
from __future__ import division

import os
import ast
import sys
import subprocess
import signal
import socket
import logging
import thread
import time
import tempfile
import math
import random
import networkx as nx
from collections import defaultdict, deque
from math import log
import sumolib
from k_shortest_paths import k_shortest_paths
from optparse import OptionParser
from bs4 import BeautifulSoup
from collections import defaultdict
from decimal import Decimal
# We need to import Python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Environment variable SUMO_HOME not defined")
    
import traci
import sumolib
class UnusedPortLock:
    lock = thread.allocate_lock()

    def __init__(self):
        self.acquired = False

    def __enter__(self):
        self.acquire()

    def __exit__(self):
        self.release()

    def acquire(self):
        if not self.acquired:
            UnusedPortLock.lock.acquire()
            self.acquired = True

    def release(self):
        if self.acquired:
            UnusedPortLock.lock.release()
            self.acquired = False

def find_unused_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
    sock.bind(('127.0.0.1', 0))
    sock.listen(socket.SOMAXCONN)
    ipaddr, port = sock.getsockname()
    sock.close()
    
    return port
def terminate_sumo(sumo):
    if sumo.returncode == None:
        os.kill(sumo.pid, signal.SIGTERM)
        time.sleep(0.5)
        if sumo.returncode == None:
            print (os.__file__)
            #os.kill(sumo.pid, signal.SIGKILL)
            time.sleep(1)
            if sumo.returncode == None:
                time.sleep(10)


def run(network, begin, end, interval):
 # parse the net
 net = sumolib.net.readNet ('D:\\import\\ttt\\UT.net.xml')
 file_open = open('RSUsLocation_xy.xml','r')
 data = file_open.read()
 soup_file = BeautifulSoup(data)
 RSU_x={}
 RSU_y={}
 dict_RSUEDGEs={}
 for RSU_tag in soup_file.findAll('poly'):
  RSU_id = RSU_tag['id']
  RSU_center = RSU_tag['center']
  RSU_x[RSU_id], RSU_y[RSU_id] = RSU_center.split(',')
  x= float(RSU_x[RSU_id])
  y= float(RSU_y[RSU_id])
  #Locate nearby edges based on the geo-coordinate
  edges = net.getNeighboringEdges(x, y, 1000) # 1000 is radius of RSU
  List_Edges=[]
  List_Edge_Id =[]
  for edge in edges:
   closestEdge, dist = edge
   List_Edges.append(str(closestEdge))
   data_id = List_Edges[0].split('id=')[1]
   Edge_id = data_id.split(' ')[0]
   List_Edge_Id.append(ast.literal_eval(Edge_id))
   del List_Edges[:]
  dict_RSUEDGEs[RSU_id]= List_Edge_Id
  logging.debug("dict_RSUEDGEs[RSU_id] [%s] = %s "% (RSU_id,dict_RSUEDGEs[RSU_id]))
                
def start_simulation(sumo, scenario, network, begin, end, interval, output):
    logging.debug("Finding unused port")
    print("Finding unused port")
    unused_port_lock = UnusedPortLock()
    unused_port_lock.__enter__()
    remote_port = find_unused_port()
    print("remote_port:{0}" .format(remote_port))
    logging.debug("Port %d was found" % remote_port)
    
    logging.debug("Starting SUMO as a server")
    
    sumo = subprocess.Popen(["D:\\E\\d\\sumo-0.25.0\\bin\\sumo.exe", "-c", "D:\\import\\ttt\\UT.sumo.cfg", "--tripinfo-output", output,"--device.emissions.probability", "1.0",  "--remote-port", str(remote_port)], stdout=sys.stdout, stderr=sys.stderr)    
    unused_port_lock.release()
            
    try:     
        traci.init(remote_port)    
        run(network, begin, end, interval)
    except Exception:
        logging.exception("Something bad happened")
    finally:
        logging.exception("Terminating SUMO")  
        terminate_sumo(sumo)
        unused_port_lock.__exit__()
        
def main():
    # Option handling
    parser = OptionParser(conflict_handler="resolve")#parser = OptionParser()
    parser.add_option("-c", "--command", dest="command", default="sumo", help="The command used to run SUMO [default: %default]", metavar="COMMAND")
    parser.add_option("-s", "--scenario", dest="scenario", default="UT.sumo.cfg", help="A SUMO configuration file [default: %default]", metavar="FILE")
    parser.add_option("-n", "--network", dest="network", default="UT.net.xml", help="A SUMO network definition file [default: %default]", metavar="FILE")    
    parser.add_option("-b", "--begin", dest="begin", type="int", default=800, action="store", help="The simulation time (s) at which the re-routing begins [default: %default]", metavar="BEGIN")
    #parser.add_option("-a", "--additional-files", dest="additional", default="UT.add.xml", help="Generate edge-based dump instead of ""lane-based dump. This is the default.", metavar="FILE")
    #parser.add_option("-a", "--additional-files", dest="additional", default="UTT.add.xml", help="Generate edge-based dump instead of ""lane-based dump. This is the default.", metavar="FILE")

    parser.add_option("-e", "--edge-based-dump", dest="edge_based_dump", action="store_true", default="True", help="Generate edge-based dump instead of ""lane-based dump. This is the default.", metavar="FILE")

    parser.add_option("-e", "--end", dest="end", type="int", default=72000, action="store", help="The simulation time (s) at which the re-routing ends [default: %default]", metavar="END")
    parser.add_option("-i", "--interval", dest="interval", type="int", default=600, action="store", help="The interval (s) at which vehicles are re-routed [default: %default]", metavar="INTERVAL")
    parser.add_option("-o", "--output", dest="output", default="mapUTT.xml", help="The XML file at which the output must be written [default: %default]", metavar="FILE")

    parser.add_option("-l", "--logfile", dest="logfile", default=os.path.join(tempfile.gettempdir(), "getedges.log"), help="log messages to logfile [default: %default]", metavar="FILE")
    (options, args) = parser.parse_args()
    
    logging.basicConfig(filename=options.logfile, level=logging.DEBUG)
    logging.debug("Logging to %s" % options.logfile)
    
    if args:
        logging.warning("Superfluous command line arguments: \"%s\"" % " ".join(args))
        
    start_simulation(options.command, options.scenario, options.network, options.begin, options.end, options.interval, options.output)
    
if __name__ == "__main__":
    main()    
    
#"--device.hbefa.probability", "1.0",