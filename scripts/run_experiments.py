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
from threading import Thread, Lock
import progressbar

ai_servers = [
    "ai1.cs.unh.edu",
    "ai2.cs.unh.edu",
    "ai3.cs.unh.edu",
    "ai4.cs.unh.edu",
    #"ai8.cs.unh.edu",
    #"ai11.cs.unh.edu",
    #"ai12.cs.unh.edu",
    #"ai13.cs.unh.edu",
    "ai14.cs.unh.edu",
    "ai15.cs.unh.edu",
]
using_servers = ", ".join(map(lambda x: x.split(".")[0], ai_servers))
working_dir = "/home/aifs2/devin/Documents/SituatdSIPP/SituatedSIPP/scripts"
program = "/home/aifs2/devin/Documents/SituatdSIPP/build_release/bin/ssipp"

timeout = str(10*60) # just to be safe

output_folder = sys.argv[2]


target_folder = {
    #"/home/aifs2/devin/Documents/SituatdSIPP/SituatedSIPP/instances/singleagent-icaps2020/warehouse/": "warehouse.xml",
    "/home/aifs2/devin/Documents/SituatdSIPP/SituatedSIPP/instances/singleagent-icaps2020/rooms/": "rooms.xml",
    #"/home/aifs2/devin/Documents/SituatdSIPP/SituatedSIPP/instances/singleagent-icaps2020/empty64x64/": "empty64x64.xml",
    #"/home/aifs2/devin/Documents/SituatdSIPP/SituatedSIPP/instances/singleagent-icaps2020/den520d/": "den520d.xml"
    }

steplim = {
    "/home/aifs2/devin/Documents/SituatdSIPP/SituatedSIPP/instances/singleagent-icaps2020/empty64x64/": "12800",
    "/home/aifs2/devin/Documents/SituatdSIPP/SituatedSIPP/instances/singleagent-icaps2020/warehouse/": "12800",
    "/home/aifs2/devin/Documents/SituatdSIPP/SituatedSIPP/instances/singleagent-icaps2020/rooms/":   "12800",
    "/home/aifs2/devin/Documents/SituatdSIPP/SituatedSIPP/instances/singleagent-icaps2020/den520d/": "31300"
    }

def find_or_create(tree, targ):
    x = tree.find(targ)
    if x:
        return x
    return ET.SubElement(tree, targ)

    
def run_exp(config, task, lookahead, learning, dm, dec, exp, uw, ni, steplimit):    
    tree = ET.parse(config)
    root = tree.getroot()
    alg = root.find("algorithm")
    aa = find_or_create(alg, "allowanyangle")
    aa.text = "false"
    conn = find_or_create(alg, "connectedness")
    conn.text = "3"
    learning_algorithm = find_or_create(alg, "learningalgorithm")
    learning_algorithm.text = learning
    look = find_or_create(alg, "fixedlookahead")
    look.text = lookahead
    dyn = find_or_create(alg, 'dynmode')
    dyn.text = dm
    decision = find_or_create(alg, "decisionalgorithm")
    decision.text = dec
    #tl = ET.SubElement(alg, "timelimit")
    tl = find_or_create(alg, "timelimit")
    tl.text = timeout
    sl = find_or_create(alg, "steplimit")
    sl.text = steplimit
    ea = find_or_create(alg, "expansionalgorithm")
    ea.text = exp
    ea = find_or_create(alg, "issituated")
    ea.text = "true"
    num_int = find_or_create(alg, "maxnumofintervalspermove")
    num_int.text = ni
    if (uw != "NA"):
        iuw = find_or_create(alg, "isunitwaitrepresentation")
        iuw.text = "true"
        u_w = find_or_create(alg, "unitwaitduration")
        u_w.text = uw
    else:
        iuw = find_or_create(alg, "isunitwaitrepresentation")
        iuw.text = "false"
    identity = "_".join([config.split("/")[-1].replace(".xml", ""), task.split("/")[-1].replace(".xml", ""), lookahead, learning, dm, dec, exp, uw, ni])
    os.mkdir(output_folder + identity)
    outconfig = output_folder + identity + "/" + config.split("/")[-1]
    tree.write(outconfig)
    obs = task.replace(".xml", "_obs.xml")
    outfile = output_folder + identity + "/" + config.split("/")[-1].replace("xml", "") + task.split("/")[-1].replace(".xml", "")  + ".xml"
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

def run_commands(commands, server, bar, lock):
    slack_client = slack.WebClient(token=sys.argv[1])
    slack_client.chat_postMessage(channel='experiments', text="Devin just started running experiments on " + server + " est: 24 hours")

    connection = Connection(server)
    for command in commands:
        connection.run(" ".join(command), hide = "both")
        lock.acquire()
        bar.update(bar.value + 1)
        lock.release()

    slack_client.chat_postMessage(channel='experiments', text="Devin: experiments finished on  " + server)


results = pd.DataFrame(columns = ["task", "lookahead", "expansion algorithm", "decision algorithm","learning algorithm", "dynmode", "solved", "solution length", "solution duration", "runtime"])
lookaheads = ["4", "8", "16", "32", "64", "128"]
learnings = ["nolearning", "dijkstralearning","plrtalearning"]
expansion = ["astar"]
decision = ["miniminbackup"]
unitwait = ["0.1", "0.5"]
numinterval = ["1"]
dynmode = ["0"]
print("Generating experiement files and folders.")
n_tasks = 0
for cfg in target_folder:
    steplimit = steplim[cfg]
    config = cfg + target_folder[cfg]
    n_tasks += len(glob(cfg+ "/*task.xml"))
total = n_tasks * len(lookaheads) * len(dynmode) * len(learnings) * len(decision) * len(expansion) * len(unitwait) * len(numinterval)
commands = []

with progressbar.ProgressBar(max_value=total) as bar:
    bar.update(0)
    for cfg in target_folder:
        steplimit = steplim[cfg]
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
                                        bar.update(len(commands))
print("Running experiements.")
with progressbar.ProgressBar(max_value=total) as bar:
    lock = Lock()
    threads = []
    for i in range(len(ai_servers)):
        c = []
        for j in range(i, len(commands), len(ai_servers)):
            c.append(commands[j])
        threads.append(Thread(target = run_commands, args = (c, ai_servers[i], bar, lock)))
        threads[-1].start()
    for i in range(len(threads)):
        threads[i].join()
        print(ai_servers[i] + " has completed!")

#results = pd.concat(outres, axis = 1).T
#results["solved"] = results["solution length"] != 0
#results.to_csv("even-even-more-results.csv")
