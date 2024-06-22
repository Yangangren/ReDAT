- [Use of multi-evaluation](#use-of-multi-evaluation)
  - [Initialize](#initialize)
  - [Step update](#step-update)
    - [**`raw_traf_veh`**: from traci](#raw_traf_veh-from-traci)
    - [**`raw_traf_light`**: from traci](#raw_traf_light-from-traci)
    - [**`agents`**: from dynamic module](#agents-from-dynamic-module)
  - [Statistical results](#statistical-results)
  - [`EvaluationResult`](#evaluationresult)
  - [Get the surrounding vehicles that in the calculation range of one agent](#get-the-surrounding-vehicles-that-in-the-calculation-range-of-one-agent)
- [Use of PDOAR Model only](#use-of-pdoar-model-only)
- [Notes for risk evaluation](#notes-for-risk-evaluation)
  - [Trajectory prediction](#trajectory-prediction)
  - [Collision detection](#collision-detection)
- [Change log:](#change-log)
  - [2023-01-12](#2023-01-12)
  - [2023-01-31](#2023-01-31)
  - [2023-02-08](#2023-02-08)
- [Best config history](#best-config-history)
  - [2023-02-08](#2023-02-08-1)


# Use of multi-evaluation

> The map_topo.py, map.py and multi_evaluation.py are all needed if you want the five dimension evaluation.
> If only concern about safety, only the podar.py is necessary, seeing the **Use of podar.py** below.

## Initialize

The evaluator must be initialized after you **open a sumo client**.

**First**, you need to organize a dict that contains basic information of agents:

```python
# including four key values: length[m], width[m], weight[ton], max_dece[m/s^2]
agents_init = {'default0': dict(length=4.5, width=1.8, weight=1.8, max_dece=7.5),
               'default1': dict(length=4.5, width=1.8, weight=1.8, max_dece=7.5)}
# agent names (keys of the init) are needed, but the four fields are not necessarry. The Evaluator will assign default values as a passenger car if they are not given:
init = {'default0': dict(),
        'default1': dict()}
# default settings:
length: float = 4.5
width: float = 1.8
weight: float = 1.8 # ton
max_dece: float = 7.5
```
> **the keys of the `angets_init` should be the same to the agent names in your sumo environment**
> This script supports multi-agents

**Then**, you can initialize a `Evaluator` instance and determine which dimension you want to evaluate:

```python
from multi_evaluation import Evaluator

...  # connect to sumo

MyEva = Evaluator(agents_init, risk=True, energy=True, comfort=True, efficiency=True, compliance=True)  # choose the dimensions
```

## Step update

At each simulation step, you need call `Evaluator.update` method to get instantaneous evaluation results.

The Evaluator keeps a instance attribute named `eva_results` in a dict: `dict[str, EvaluationData]`(see the final of this ReadMe)

```python
# way one ======================
ins_res = MyEva.update(raw_traf_veh=...,  # surrouding vehicle info
                       raw_traf_light=...,  # traffic light info
                       agents=...)  # ego info
# way two ======================
# run this line in your step method
MyEva.update(raw_traf_veh=...,  # surrouding vehicle info
             raw_traf_light=...,  # traffic light info
             agents_dyn=...)  # ego info
# and get the results by the instance attribute at somewhere
ins_res = MyEva.eva_results
```

Here are three inputs needed, and they can be got from traci subscription method and agent dynamic module.

### **`raw_traf_veh`**: from traci

```python
#  these fields must be contained in the subscription: 
junction_ID = traci.junction.getIDList()[0]  # get random junction ID
traci.junction.subscribeContext(junction_ID,
                                traci.constants.CMD_GET_VEHICLE_VARIABLE,
                                1000000.,
                                [traci.constants.VAR_POSITION,  # xy
                                 traci.constants.VAR_ANGLE, # 67, heading
                                 traci.constants.VAR_SPEED,  
                                 traci.constants.VAR_ACCELERATION,
                                 traci.constants.VAR_LENGTH,
                                 traci.constants.VAR_WIDTH,
                                 # following is the need for agents
                                 traci.constants.VAR_LANE_INDEX, # 82
                                 traci.constants.VAR_ROAD_ID, # 80
                                 traci.constants.VAR_LANE_ID, # 81
                                 traci.constants.VAR_LANEPOSITION, # 86
                                 traci.constants.VAR_LANEPOSITION_LAT, # 184
                                 ],
                                 0, 10000000)
# =================================================
# sometimes, the configuration in SUMO uses another one method to generate pedestrians, 
# so an additional subcription is needed to get person information
# =================================================
traci.junction.subscribeContext(junction_ID,
                                traci.constants.CMD_GET_PERSON_VARIABLE,  # person
                                1000000.,
                                [traci.constants.VAR_POSITION,
                                traci.constants.VAR_ANGLE, # 67
                                traci.constants.VAR_SPEED,
                                # traci.constants.VAR_ACCELERATION,  <-- invalid
                                traci.constants.VAR_LENGTH,
                                traci.constants.VAR_WIDTH,
                                # traci.constants.VAR_LANE_INDEX, # 82  <-- invalid
                                # traci.constants.VAR_ROAD_ID, # 80  <-- invalid
                                # traci.constants.VAR_LANE_ID, # 81  <-- invalid
                                # traci.constants.VAR_LANEPOSITION, # 86  <-- invalid
                                # traci.constants.VAR_LANEPOSITION_LAT, # 184  <-- invalid
                                ],
                                0, 10000000)

# get the results, including ego vehicle information
raw_traf_veh = traci.junction.getContextSubscriptionResults(junction_ID)
```
> The coordinate system transfer will be treated by this script

> You can directly add above codes in you pojects, ensuring the `junction_ID` is different with that you used in your subscription.

### **`raw_traf_light`**: from traci

```python
# you need to subscribe all the traffic light information
tempTLlist = traci.trafficlight.getIDList()
for TLid in tempTLlist:
traci.trafficlight.subscribe(TLid,
                            [traci.constants.TL_CURRENT_PHASE,
                            traci.constants.TL_RED_YELLOW_GREEN_STATE,
                            traci.constants.TL_CONTROLLED_LINKS],
                            0, 10000000)

# get traffic light information
raw_traf_light = traci.trafficlight.getAllSubscriptionResults()
```

### **`agents`**: from dynamic module

```python
# all the eight fields are needed
agents = {'default0': dict(x=...,  # center, different with sumo
                           y=...,  # center, different with sumo
                           phi=...,  # rad, heading, to right=0, different with sumo
                           u=...,  # speed m/s
                           acc_lon=...,  # m/s^2
                           acc_lat=...,  # m/s^2
                           fuel_rate=...,  # L/s
                           mileage=...),  # km
          'default1': dict(x=..., y=..., phi=..., speed=..., acc_lon=...,
                           fuel_rate=..., mileage=...,)}
```

> Lack of these fields will not generate program errors, but the evaluation results will be affected:
> `x, y, phi, u, acc_lon` will affect the trajectory prediction, then affect risk evaluation.
> `acc_lon, acc_lat` will affect the comfort evaluation
> `fuel_rate, mileage` will affect the energy efficiency evaluation

> The update method have a filter to select the surrounding vehicles in a range:
> - if ego vehicle not in a intersection, only surrounding vehicles whose relative heading to ego vehicle is less than 30 degree, and relative Manhattan distance to ego vehicle is < 50m,  will be considered
> - else, all the surrounding vehicles whose relative Manhattan distance to ego vehicle is < 50m,  will be considered
> - you also can change the two parameters by modify the variables in the evaluator init: `self.manhattan_dis` and `self.relative_angle`

## Statistical results

When finishing the simulaion, run `Evaluator.update_statistical` to get the statistical results:

```python
sta_res = MyEva.update_statistical()
```
The statistical results will also be updated in the data structure `EvaluationResult`

## `EvaluationResult`

```python
MyEva.eva_results: Dict[str, EvaluationData]  # the key is agent name

@dataclass
class EvaluationData:
    # instantaneous eval, update each step
    eva_risk: float = None  # current risk
    act_collision: bool = None  # if collided at current
    pred_collision: bool = None  # if predicted collided if maintain current state
    eva_energy: float = None 
    eva_comfort: float = None
    eva_efficiency: float = None
    eva_compliance: int = None
    
    # statistical eval [after simulation]
    st_risk: float = 0.  # average risk
    st_coll: int = 0  # total number of collision 
    st_comf: float = 0.  # average comfort 
    st_comf_lvl: int = 0  # total num, steps comfort > level 3 recorded
    st_traf: float = 0.  # average 
    st_ener: float = 0. # L/100km
    st_regu: float = 0. # sum <= 0, 0 is the best
```

## Get the surrounding vehicles that in the calculation range of one agent

The `Evaluator` class keeps a attribute named `vehicles`.

This variable contains all the information when calculating risk.

You can get one agent's surrounding vehicle name and instantaneous risk values brought by them using:

```python
MyEva.vehicles.ego['default0'].risk2obj
# return:
# {'surr_name1': risk1, 'surr_name2': risk2}
```

Other evaluation indicators can also be got, and the xy, speed, trajectory prediction of surrouding vehicles are also contained in this variable.


# Use of PDOAR Model only
The PODAR model can be used as following:
``` python
from podar import PODAR
# [step 0]
podar = PODAR()
# for each ego vehicles:
# [step 1] set static parameters (only once)
podar.add_ego(name=_name, length=l, width=w, mass=ms, max_dece=md) 
# [step 2] set dynamic information
podar..update_ego(name=ego_name, x0=_ego.x, y0=_ego.y, speed=_ego.u, phi0=_ego.phi, a0=_ego.acc_lon, mileage=_ego.mileage)  # dynamic information
# [step 3] here, the ego_surr_list determines which surrounding vehicle will participate the risk calculation
podar.ego_surr_list = {'ego_1_name': ['ego1_surr_veh_1_id', 'ego1_surr_veh_2_id', ...],
                       'ego_2_name': ['ego2_surr_veh_1_id', 'ego2_surr_veh_2_id', ...]}

# [step 4] for surrounding vehicles:
podar.add_obj(name=_ov.id, x0=_ov.x, y0=_ov.y, speed=_ov.u, phi0=_ov.phi, a0=_ov.acc_lon, length=_ov.length, width=_ov.width)

# [step 5] get risk:
risk, if_real_collision, if_predicted_collision = self.vehicles.estimate_risk(ego_name)
# [step 6] every step, you need to reset the podar to clear the surrounding vehicles information
podar.reset()

# in each cycle, you need to re-run [steps 2-6]

# [step 7] after finish the task, get statistical results:
st_risk = np.mean(self.vehicles.ego[_name].st_risk)  # mean risk, you can use other method
st_coll = podar.ego[_name].st_coll  # count of collision
```
To change the parameters in PODAR, you can modify the values in dict `config`, at the front of the podar.py.
```python
config = dict(
        description='',
        step_interval=0.1,  # trajectory prediction step interval, second
        traj_pre_ver='omit_min_action',  # no use 
        pred_hori=4,  # prediction horizon
        A=1.04,  # determine the attenuation curve in temporal dimension, bigger value means quick decrease
        B=1.94,  # determine the attenuation curve in temporal dimension, bigger value means quick decrease
        Alpha=0.4,  # balance coef for delta_v and abs_v, recommende not to change
        stochastic=False,  # no use 
        consider_max_dece=False,  # recommende not to change
        distance_method='shapely', # use Contour distance, or 'circle': circle distance
        damage_form='fix damage', # or 'kinetic form', the way to calculate damage
        delta_v_method='norm',  # no use
        use_delta_v_vector=False,  # if use vector method instead of abs(delta v)
        relative_direction_method='obj one point'  # no use
    )
```

# Notes for risk evaluation

## Trajectory prediction

The trajectory prediction uses a uniform acceleration recursive model and considring the heading changes. You can modify it by re-writing method `traj_predition`, and keep the codes at `Line 293` in `multi_evaluation.py`.

## Collision detection

This script uses the outer contour distances by assuming vehicles a rectangle, instead of using the six/four/two-circles method, via `shapely` lib of python to detect the collision. So, it can not be used in the model-based RL.

If you want to modify it, you need to re-write codes at `Line 349-350` in `multi_evaluation.py`. The trajectroy prediction results can be found by call `Veh_obj.x_pred, Veh_obj.y_pred, Veh_obj.phi_pred, Veh_obj.width, Veh_obj.length`.

---
> If have any question, please feel free to contact Chen Chen

---
# Change log:
## 2023-01-12
- modify the `class Vehicles` to `class PODAR`
- separate the PODAR model as a independent py file (*podar.py*)
- now the parameters can be customized by modifying the `config` in the start of *podar.py*
- update the trajectory prediction method
## 2023-01-31
- now the default configure is to use the $abs(\delta v)$ instead of the vector method, and the $\alpha$ is set to 0.5
## 2023-02-08
- add damage form: *damage loglog*, letting damage be not sensitive to speed
- add render function in *podar.py*: `podar_render(frame: PODAR, ego_name: str = None)`
- adjust the fix damage of different types of object: tru: 3->1.5, bic and ped: 2.5->1.2
- adjust the length judgement for object types: ped: length<1, bic: length<2.5, car: length<7.5, tru: length>7.5
- subversion 1: the best config adjust to: 2023-02-08(version 1), risk range is [0,2.2); dangerous threshold=1.2
- subversion 2: the best config adjust to: 2023-02-08(version 2), risk range is [0,2.1); dangerous threshold=1.16
  - use the center point of vehicles to calculate $\delta v$ with vector method
  - use the 'signal plus' method for velocity calculation in damage




# Best config history
## 2023-02-08
**subversion 1**
``` python
config = dict(
        description='use new A and B, use abs delta_v',
        step_interval=0.1,  # trajectory prediction step interval, second
        traj_pre_ver=None,  # no use 
        pred_hori=4,  # prediction horizon
        A=1.04, # 0.17,  # determine the attenuation curve in temporal dimension
        B=1.94, # 0.58,  # determine the attenuation curve in temporal dimension
        Alpha=0.5,  # balance coef for delta_v and abs_v, recommende not to change
        stochastic=False,  # no use 
        consider_max_dece=False,  # recommende not to change
        distance_method='shapely', # or 'circle', use manhatton distance instead of Euclidean distance
        damage_form='fix damage loglog', # or 'kinetic form', the way to calculate damage
        delta_v_method='norm',  # no use        
        use_delta_v_vector=False,  # direct use abs(delta v)
        relative_direction_method='obj one point'  # no use
    )
```
**subversion 2**
``` python
config_center = dict(
        description='use average A and B, set alpha to 0.5, test the map with different distances, use center point for delta_v, for PCD_model_verification\PODAR_test\collision_research\case.ipynb',
        date='2023.02.09',
        step_interval=0.1,
        traj_pre_ver=None,
        pred_hori=4,
        A=1.04,
        B=1.94,
        Alpha=0.5,
        stochastic=False,
        consider_max_dece=False,
        distance_method='shapely',
        damage_form='fix damage loglog',  # fix damage loglog
        delta_v_method='norm',
        use_delta_v_vector=True,
        relative_direction_method='both center',
        velocity_method='signal plus'
    )
```