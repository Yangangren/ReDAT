# ReDAT
Learn a Robust Policy for Real-world Driving with Adversarial Reinforcement Learning

## Requires
1. Ubuntu 18.04 LTS is necessary.
2. [SUMO](https://sourceforge.net/projects/sumo/files/sumo/) must be installed with version from 1.13.0 to 1.16.0.
3. Python 3.8.
4. The installation path must be in English.

## Installation
```bash
# Please make sure not to include Chinese characters in the installation path, as it may result in a failed execution.
# clone repository
git clone https://github.com/Yangangren/ReDAT.git
cd ReDAT
# create conda environment
conda env create -f xxxx.yml
conda activate ReDAT
# install ReDAT
pip install -e.
```

## Train
Train the policy by running:
```bash
cd algorithm
#Train a policy at the urban intersection
python train_script.py
```
After training, the results will be stored in the "algorithm/results" folder.

## Simulation
Test the specified policy in the "env_build/hier_decision" folder:
```bash
cd env_build/hier_decision
```
Then modify the path of trained policy and choose the appropriate iteration number in "hier_decision.py" and run it:
```bash
python hier_decision.py
```
After running, the simulation snapshots and indicator curve figures will be stored in the "/env_build/hier_decision/results/" folder.

## Real Vehicle Experiments
After obtaining a satisfactory policy, it can be deployed in a full-size vehicle to drive at the urban intersection. The experimental result is showed in "Real_experiment_video".

## Acknowledgment
We would like to thank all members in Intelligent Driving Laboratory (iDLab), School of Vehicle and Mobility, Tsinghua University for making excellent contributions and providing helpful advices for ReDAT.