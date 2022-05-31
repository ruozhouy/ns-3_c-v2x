#####################################################
# Running different experiments and saving the logs
import os, sys, os.path
from subprocess import Popen
import time

# 0. Global variables with default values
# -> Execution setting
nProc = 10     # number of concurrent processes to run
sleepTime = 1  # seconds to sleep for checking on a process again

# -> Exp parameters
idealScheduled = 0
numVeh = 10
numCam = 10
sizeSubchannel = 25
numSubchannel = 1

def setExpParams(exp_params):
    global idealScheduled, numVeh, numCam, sizeSubchannel, numSubchannel
    if 'idealScheduled' in exp_params:
        idealScheduled = exp_params['idealScheduled']
    if 'numVeh' in exp_params:
        numVeh = exp_params['numVeh']
    if 'numCam' in exp_params:
        numCam = exp_params['numCam']
    if 'sizeSubchannel' in exp_params:
        sizeSubchannel = exp_params['sizeSubchannel']
    if 'numSubchannel' in exp_params:
        numSubchannel = exp_params['numSubchannel']

data_dir = 'v2x_data_2/'
if (not os.path.isdir(data_dir)):
    os.mkdir(data_dir)

# 1. Generate default parameters
# Format: ./waf --run "v2x_communication_example --numVeh=20 --numCam=5 --sizeSubchannel=5 --numSubchannel=1 --idealScheduled=1"
def genCmd():
    cmd = [ './waf', '--run' ]
    exp_params = 'scratch/v2x_communication_example'
    exp_params += ' --numVeh=' + str(numVeh)
    exp_params += ' --numCam=' + str(numCam)
    exp_params += ' --sizeSubchannel=' + str(sizeSubchannel)
    exp_params += ' --numSubchannel=' + str(numSubchannel)
    exp_params += ' --idealScheduled=' + str(idealScheduled)
    exp_params = '' + exp_params + ''
    cmd.append( exp_params )
    return cmd

# 2. Generate output file name
def genOutFn():
    fn = data_dir
    fn += '_' + str(idealScheduled)
    fn += '_' + str(numVeh)
    fn += '_' + str(numCam)
    fn += '_' + str(sizeSubchannel)
    fn += '_' + str(numSubchannel)
    fn += '.txt'
    return fn

# 3. Generate set of experiment settings
exp_sets = []
for expNumVeh in [5]:
    maxNumCam = int(100 / expNumVeh)
    for expNumCam in range(1, 31):
        for iSch in [0, 1]:
            exp_setting = {
                'numVeh': expNumVeh,
                'numCam': expNumCam,
                'idealScheduled': iSch,
            }
            exp_sets.append(exp_setting)
print(exp_sets)

# 4. Run the experiments
procs = []
for exp_param in exp_sets:
    print('- Executing experiment:', exp_param)
    setExpParams(exp_param)

    # Remove finished processes
    while len(procs) >= nProc:
        time.sleep(sleepTime)
        finof = [ of for p, of in procs if p.poll() != None ]   # poll != None means finished
        procs = [ (p, of) for p, of in procs if p.poll() == None ]  # poll = None means running
        for of in finof:
            of.close()

    # Run experiment
    cmd = genCmd()
    #print(cmd)
    ofn = genOutFn()
    of = open(ofn, 'w')
    p = Popen(cmd, stdout=of, stderr=of)
    procs.append( (p, of) )

# Remove finished processes
while procs != []:
    time.sleep(sleepTime)
    finof = [ of for p, of in procs if p.poll() != None ]   # poll != None means finished
    procs = [ (p, of) for p, of in procs if p.poll() == None ]  # poll = None means running
    for of in finof:
        of.close()

# 5. Finished
print("!!! All Experiments Finished !!!")
    


















