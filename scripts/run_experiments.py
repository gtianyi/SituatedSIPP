import xml.etree.ElementTree as ET
from glob import glob
import subprocess
import os
import pandas as pd
from multiprocessing import Pool
import shutil

program = "../../build_release/bin/ssipp"

timeout = 30

target_folder = {
    "../instances/singleagent-icaps2020/empty64x64/": "empty64x64.xml",
    "../instances/singleagent-icaps2020/warehouse/": "warehouse.xml",
    "../instances/singleagent-icaps2020/rooms/": "rooms.xml",
    "../instances/singleagent-icaps2020/den520d/": "den520d.xml"
                }

results = pd.DataFrame(columns = ["task", "lookahead", "learning algorithm", "dynmode", "solved", "solution length", "solution duration", "runtime"])
lookaheads = ["10", "100", "1000"]
learnings = ["dijkstralearning", "plrtalearning"]
dynmode = ["0", "1"]
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
    
def run_exp(config, task, lookahead, learning, dm):    
    tree = ET.parse(config)
    root = tree.getroot()
    alg = root.find("algorithm")
    learning_algorithm = alg.find("learningalgorithm")
    learning_algorithm.text = learning
    look = alg.find("fixedlookahead")
    look.text = lookahead
    dyn = ET.SubElement(alg, 'dynmode')
    dyn.text = dm
    outconfig = dm + config.split("/")[-1]
    tree.write(outconfig)
    obs = task.replace(".xml", "_obs.xml")
    outfile = "outfiles/" + config.split("/")[-1].replace("xml", "") + task.split("/")[-1].replace("xml", "") + lookahead + learning + dm + ".xml"
    command = [program, task, config, outconfig, obs, outfile]
    try:
        subprocess.run(command, timeout = timeout)
        res = parse_output(outfile)
    except:
        print(" ".join(command))
        res = (float("inf"),timeout, 0)
    return pd.Series(index = results.columns, data = (task, lookahead, learning, dm, True, res[2], res[0], res[1]))

c = 0
exp_results = []
pool = Pool(4)
for cfg in target_folder:
    config = cfg + target_folder[cfg]
    tasks = glob(cfg+ "/*task.xml")
    for lookahead in lookaheads:
        for task in tasks:
            for learning in learnings:
                for dm in dynmode:
                    print(lookahead, task, learning, dynmode)
                    exp_results.append(pool.apply_async(run_exp, (config, task, lookahead, learning, dm)))
        break
    break
outres = []
for res in exp_results:
    outres.append(res.get())

results = pd.concat(outres, axis = 1).T
results["solved"] = results["solution length"] != 0
results.to_csv("even-even-more-results.csv")