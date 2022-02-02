import sys
import xml.etree.ElementTree as ET
from glob import glob
import subprocess
import os
import pandas as pd
from multiprocessing import Pool
import shutil
import slack
import socket
from fabric import Connection
from threading import Thread

ai_servers = [
    "ai1.cs.unh.edu",
    "ai2.cs.unh.edu",
    "ai3.cs.unh.edu",
    "ai4.cs.unh.edu",
    #"ai8.cs.unh.edu",
    #"ai11.cs.unh.edu",
    "ai12.cs.unh.edu",
    "ai13.cs.unh.edu",
    "ai14.cs.unh.edu",
    "ai15.cs.unh.edu",
]
using_servers = ", ".join(map(lambda x: x.split(".")[0], ai_servers))

program = "../../build_release/bin/ssipp"

timeout = str(10*60) # just to be safe

target_folder = {
    "../instances/singleagent-icaps2020/empty64x64/": "empty64x64.xml",
    "../instances/singleagent-icaps2020/warehouse/": "warehouse.xml",
    "../instances/singleagent-icaps2020/rooms/": "rooms.xml",
    "../instances/singleagent-icaps2020/den520d/": "den520d.xml"
    }

steplim = {
    "../instances/singleagent-icaps2020/empty64x64/": "12800",
    "../instances/singleagent-icaps2020/warehouse/": "12800",
    "../instances/singleagent-icaps2020/rooms/":   "12800",
    "../instances/singleagent-icaps2020/den520d/": "31300"
    }



def parse_output(filepath):
    tree = ET.parse(filepath)
    root = tree.getroot()
    try:
        log = root.find("log")
        agent = log.find("agent")
        path = agent.find("path")
        return float(path.attrib["duration"]),float(path.attrib["runtime"]), len(path)
    except:
        return float("inf"),float("inf"), 0
    
def run_exp(config, task, lookahead, learning, dm, dec, exp, uw, ni, steplimit):    
    tree = ET.parse(config)
    root = tree.getroot()
    alg = root.find("algorithm")
    learning_algorithm = alg.find("learningalgorithm")
    learning_algorithm.text = learning
    look = alg.find("fixedlookahead")
    look.text = lookahead
    dyn = ET.SubElement(alg, 'dynmode')
    dyn.text = dm
    decision = alg.find("decisionalgorithm")
    decision.text = dec
    tl = ET.SubElement(alg, "timelimit")
    tl.text = timeout
    sl = ET.SubElement(alg, "steplimit")
    sl.text = steplimit
    ea = ET.SubElement(alg, "expansionalgorithm")
    ea.text = exp
    num_int = ET.SubElement(alg, "maxnumofintervalspermove")
    num_int.text = ni
    if (uw != "NA"):
        iuw = ET.SubElement(alg, "isunitwaitrepresentation")
        iuw.text = "true"
    u_w = ET.SubElement(alg, "unitwaitduration")
    u_w.text = uw
    identity = "_".join([config.split("/")[-1].replace(".xml", ""), task.split("/")[-1].replace(".xml", ""), lookahead, learning, dm, dec, exp, uw, ni])
    os.mkdir("outfiles_test/" + identity)
    outconfig = "outfiles_test/" + identity + "/" + config.split("/")[-1]
    tree.write(outconfig)
    obs = task.replace(".xml", "_obs.xml")
    outfile = "outfiles_test/" + identity + "/" + config.split("/")[-1].replace("xml", "") + task.split("/")[-1].replace(".xml", "")  + ".xml"
    command = [program, task, config, outconfig, obs, outfile]
    return command
    #print(command)
    #subprocess.run(command)
    #try:
    #    subprocess.run(command)
    #    #res = parse_output(outfile)
    #except:
    #    print(" ".join(command))
        #res = (float("inf"),timeout, 0)
    #return pd.Series(index = results.columns, data = (task, lookahead, learning, dm, True, res[2], res[0], res[1]))

def run_commands(commands, server):
    with slack.WebClient(token=sys.argv[1]) as slack_client:
        slack_client.chat_postMessage(channel='experiments', text="Devin just started running experiments on " + server + " est: 24 hours")
    connection = Connection(server)
    for command in commands:
        connection.run(command, hide = "both")
        break
    with slack.WebClient(token=sys.argv[1]) as slack_client:
        slack_client.chat_postMessage(channel='experiments', text="Devin: experiments finished on  " + server)


results = pd.DataFrame(columns = ["task", "lookahead", "expansion algorithm", "decision algorithm","learning algorithm", "dynmode", "solved", "solution length", "solution duration", "runtime"])
lookaheads = ["40", "80", "320", "1280"]
learnings = ["nolearning","dijkstralearning", "plrtalearning"]
expansion = ["astar"]
decision = ["miniminbackup"]
unitwait = ["0.1", "1"]
numinterval = ["100"]
dynmode = ["0"]

commands = []
for cfg in target_folder:
    steplimit = steplim[cfg]
    if sys.argv[2] not in cfg:
        continue
    config = cfg + target_folder[cfg]
    tasks = glob(cfg+ "/*task.xml")
    for lookahead in lookaheads:
        for task in tasks:
            for learning in learnings:
                for dm in dynmode:
                    for dec in decision:
                        for exp in expansion:
                            for uw in unitwait:
                                for ni in numinterval:
                                    commands.append(run_exp(config, task, lookahead, learning, dm, dec, exp, uw, ni, steplimit))

threads = []
for i in range(len(using_servers)):
    c = []
    for j in range(i, len(commands), len(using_servers)):
        c.append(commands[j])
    threads.append(Thread(target = run_commands, args = (c, using_servers[i])))
    threads[-1].start()

for i in range(len(threads)):
    threads[i].join()

#results = pd.concat(outres, axis = 1).T
#results["solved"] = results["solution length"] != 0
#results.to_csv("even-even-more-results.csv")
