# This Python file uses the following encoding: utf-8
# coding=utf-8

# ===================================================================
# update the evaluation results.
# author: Chen Chen
# 2022.05.24 --------------
# add a feature to Vehi_obj, to let surrounding vehicle save the risk results to each ego car
# -------------------------
# ===================================================================

from math import sin, cos, pi, sqrt, atan2
import numpy as np
from shapely.geometry import Polygon
from typing import Dict, List, Tuple
from dataclasses import dataclass, field
from multi_evaluation_podar import map_topo
from multi_evaluation_podar.map import Map
from multi_evaluation_podar.podar import PODAR, Veh_obj
import copy

@dataclass
class EvaluationData:  # 全局评价结果
    # instantaneous eval
    eva_risk: float = None  # 当前风险值
    act_collision: bool = None  # 当前是否发生碰撞
    pred_collision: bool = None  # 预测是否发生碰撞
    eva_energy: float = None  # 节能性
    eva_comfort: float = None  # 舒适性
    eva_efficiency: float = None  # 高效性
    eva_compliance: int = None  # 合规性
    
    # statistical eval [after simulation]
    st_risk: float = 0.  # risk recorde
    st_coll: int = 0  # num of collision 
    st_comf: float = 0.  # average comfort 
    st_comf_lvl: int = 0  # steps comfort > level 3 recorde
    st_traf: float = 0.  # average 
    st_ener: float = 0. # sum
    st_regu: float = 0. # sum


@dataclass
class TrafficVeh:
    id: str = '0'
    length: float = 4.8
    width: float = 2.0
    weight: float = 0.0  # mass

    x: float = 0.0
    y: float = 0.0
    phi: float = 0.0  # heading angle, rad, consistent with dynamics
    u: float = 0.0
    v: float = 0.0
    acc_lon: float = 0.0  # acceleration_vertical
    acc_lat: float = 0.0  # acceleration_lateral
    mileage: float = 0.0
    
    fuel_rate: float = -1
    lane_index: int = 0
    road_id: str = '0'
    lane_id: str = '0'
    lane_position: float = 0.0
    lateral_position: float = 0.0
    max_dec: float = 7.0

    def update(self, **kwargs):
        """update related parameters during calculation
        """
        for key, val in kwargs.items():
            assert key in vars(self), '{} is not a class attr'.format(key)
            exec("self.{0}=val".format(key), {'self': self, 'val': val})

@dataclass
class TrafficLight:
    phase: int = 0  # current phase, 0,1,2...
    state: str = ''  # red_yellow_green state, 1-green; 2-red; 3-yellow



class Evaluator(object):
    def __init__(self, init: Dict[str, dict], risk=True, energy=True, comfort=True, efficiency=True, compliance=True):
        """
        [initial]
        Define the ego_name list
        """
        Map().update()
        self.init: Dict[str, dict] = init
        self.reset()
        self.flag_risk = risk
        self.flag_energy = energy
        self.flag_comfort = comfort
        self.flag_efficiency = efficiency
        self.flag_compliance = compliance
        self.manhattan_dis = 50
        self.relative_angle = 1 / 6 * pi
            
    def update(self, raw_traf_veh: dict, raw_traf_light: dict, agents_dyn: dict) -> Dict[str, EvaluationData]:
        """
        [simplified evaluation update function]
        Procedure:
        1. clear trajectory prediction information of last step in data_module
        2. get current ego vehicle and surrounding vehicle information
        3. check the surrounding vehicles that locates in the evaluation range
        4. get risk evaluation and other results
        5. clear past information
        6. save risk value to data_module
        """
        self.raw_traf_veh, self.raw_traf_light, self.raw_agents = raw_traf_veh, raw_traf_light, agents_dyn
        self.api_wrap()
        # 清除上一时刻轨迹预测信息 (Procedure 1)  TODO: 不再存储轨迹预测信息
        # 获取上一时刻自车和周车信息 (Procedure 2)
        self.ov_data = self.traffic_vehs
        for name, info in self.raw_agents.items():
            self.ego_data[name].update(**info)
            self.ov_data[name] = self.ego_data[name]
        # 初始化车辆信息  (Procedure 3)
        eff_vehicle_list, spe_ego_sur_veh = self.vehicle_init()
        # 计算风险  (Procedure 4)
        self.vehicles.ego_surr_list = spe_ego_sur_veh
        for ego_name in self.ego_name_list:  # 更新自车信息
            _ego = self.ego_data[ego_name]
            self.vehicles.update_ego(name=ego_name, x0=_ego.x, y0=_ego.y, speed=_ego.u, phi0=_ego.phi, a0=_ego.acc_lon, mileage=_ego.mileage)
            
        for ov_name in eff_vehicle_list:  # 添加周车信息，同时计算安全性
            _ov = self.ov_data[ov_name]
            self.vehicles.add_obj(name=_ov.id, x0=_ov.x, y0=_ov.y, speed=_ov.u, phi0=_ov.phi, a0=_ov.acc_lon,
                                  length=_ov.length, width=_ov.width)
        
        for ego_name in self.ego_name_list:
            saved_ego, _ego =self.vehicles.ego[ego_name], self.ego_data[ego_name]
            if self.flag_compliance:
            # 合规性  (Procedure 5)
                self.regulatory_compliance(saved_ego, _ego)
                self.eva_results[ego_name].eva_compliance = saved_ego.regulatory_penalty
            # 舒适性
            if self.flag_comfort:
                self.comfort(saved_ego, _ego)
                self.eva_results[ego_name].eva_comfort = saved_ego.st_comf[-1]
            # 经济性
            if self.flag_energy:
                saved_ego.st_ener += _ego.fuel_rate
                self.eva_results[ego_name].eva_energy = _ego.fuel_rate
            # 效率性
            if self.flag_efficiency:
                if (len(saved_ego.surr_speed) != 0) and (np.sum(saved_ego.surr_speed) != 0):
                    trff_ef = _ego.u / np.mean(saved_ego.surr_speed)
                else:
                    trff_ef = 1
                saved_ego.st_traf.append(trff_ef)
                self.eva_results[ego_name].eva_efficiency = trff_ef     
            # 安全性
            if self.flag_risk:
                risk, rc, pc = self.vehicles.estimate_risk(ego_name)
                self.eva_results[ego_name].eva_risk, self.eva_results[ego_name].act_collision, self.eva_results[ego_name].pred_collision = risk, rc, pc                    
        
        # 清除信息  (Procedure 5)
        vehicles_record = copy.deepcopy(self.vehicles)
        self.vehicles.reset()
        
        # 返回数据 (Procedure 5)
        return self.eva_results, vehicles_record

    def update_statistical(self):
        for _name in self.ego_name_list:  # TODO: add these info after final simu step to data
            self.eva_results[_name].st_risk = np.mean(self.vehicles.ego[_name].st_risk)
            self.eva_results[_name].st_coll = self.vehicles.ego[_name].st_coll
            self.eva_results[_name].st_comf = np.mean(self.vehicles.ego[_name].st_comf)
            self.eva_results[_name].st_comf_lvl = self.vehicles.ego[_name].st_comf_lvl
            self.eva_results[_name].st_traf = np.mean(self.vehicles.ego[_name].st_traf)
            self.eva_results[_name].st_ener = np.sum(self.vehicles.ego[_name].st_ener) / self.vehicles.ego[_name].mileage * 100
            self.eva_results[_name].st_regu = np.sum(self.vehicles.ego[_name].st_regu)
            ...
        return self.eva_results
            
    def reset(self):
        self.ego_name_list = list(self.init.keys())
        self.ego_data: dict[str, TrafficVeh] = {}
        self.ov_data: dict[str, TrafficVeh] = {}
        self.vehicles = PODAR()
        self.eva_results: dict[str, EvaluationData] = {}
        for _name in self.ego_name_list:
            l = self.init[_name]['length'] if 'length' in self.init[_name].keys() else 4.5
            w = self.init[_name]['width'] if 'width' in self.init[_name].keys() else 1.8
            ms = self.init[_name]['mass'] if 'mass' in self.init[_name].keys() else 1.8
            md = self.init[_name]['max_dece'] if 'max_dece' in self.init[_name].keys() else 7.5
            self.vehicles.add_ego(name=_name, length=l, width=w, mass=ms, max_dece=md)
            self.eva_results[_name] = EvaluationData()
            self.ego_data[_name] = TrafficVeh(id=_name, length=l, width=w, weight=ms)
        print('[EVALUATION]: Evaluation module has been initialized.')
            
    def vehicle_init(self) -> Tuple[List[str], Dict[str, List[str]]]:
        """
        [check the vehicles located in evaluation range]
        Contents:
        1. initialize the global variables of ego vehicles
        2. select the surrounding vehicles that located in the range of ego vehicles
            range: manhattan distance < 50 [and relative heading less than 30 degree on multiple lanes]
            and then initialize their global variables
        3. delete the information in global variables of vehicles that quit the range

        Return:
            used_vehicle_list (List[str]): sum of surrounding vehicle name list that around the ego vehicles
            spe_ego_sur_veh (Dict[str, List[str]]]): for each ego vehicle, its surrounding vehicle name list
        """
        used_vehicle_list = []  # 所有周车name列表，不包含自车
        spe_ego_sur_veh = {}  # 每个自车在计算范围内的周车name列表，包含其他自车
        for ego_name_ in self.ego_name_list:  # 遍历自车
            ego_angle = self.ego_data[ego_name_].phi
            tmp_list = []
            for ov_name, ov_info in self.ov_data.items():  # (Procedure 2)
                if ov_name == ego_name_: continue  # 如果是自车，跳过
                ov_angle = ov_info.phi
                # 曼哈顿距离>50m,剔除
                if abs(self.ego_data[ego_name_].x - ov_info.x) + abs(
                        self.ego_data[ego_name_].y - ov_info.y) > self.manhattan_dis: continue
                # if (self.ego_data[ego_name_].road_id != ':') and \
                #         (11 / 6 * pi > abs(ov_angle - ego_angle) > self.relative_angle):  # 不在路口且夹角超过30度
                #     continue
                if ov_name not in used_vehicle_list and ov_name not in self.ego_name_list: 
                    used_vehicle_list.append(ov_name)  # 非自车加入总周车列表
                tmp_list.append(ov_name)  # 加入当前自车周车列表
                self.vehicles.ego[ego_name_].surr_speed.append(ov_info.u)
            spe_ego_sur_veh[ego_name_] = tmp_list

        return (used_vehicle_list, spe_ego_sur_veh)

    def regulatory_compliance(self, saved_ego: Veh_obj, current_ego: TrafficVeh):
        if (saved_ego.prev_laneid == '') or (saved_ego.prev_laneid == current_ego.lane_id): 
            return  # 仿真初始，或者无换道、进入交叉口的行为
        # 出现车道变化：
        elif (saved_ego.prev_roadid == current_ego.road_id):  
            # 但路段没变化，即仍处于同一基本路段上
            rest_length = map_topo._LInfo[current_ego.lane_id]['length'] - current_ego.lane_position
            if (rest_length < 30) and (current_ego.lane_id in map_topo._LS.keys()):  # 在信控交叉口30m范围内换道
                saved_ego.regulatory_penalty += -2  # 违规换道            
            saved_ego.prev_laneid = current_ego.lane_id  # 更新记录的车道信息
        elif (saved_ego.prev_roadid != current_ego.road_id):
            # 进入了新的路段
            if (current_ego.road_id[0] == ':'):  # 进入交叉口
                if current_ego.lane_id in map_topo._LS.keys():  # 从受控车道驶入交叉口
                    control_info = map_topo._LS[saved_ego.prev_lane]
                    for con_info in control_info:  # 循环列表中的每个[LSID, i, target_roadID]对
                        if self.traffic_lights[con_info[0]].state[con_info[1]] in 'Rr':  # 如果状态是红灯，说明闯红灯了
                            saved_ego.regulatory_penalty += -6  # 闯红灯
            else:  # 从交叉口进入基本路段
                last_roadID, last_lane_index = laneIDstr_resolve(saved_ego.prev_laneid, 'all')
                if current_ego.road_id not in map_topo._RL2RaL['passenger']\
                    [last_roadID][int(last_lane_index)].keys():  # 如果当前的路段ID不在之前车道能去的地方，就违规了
                    saved_ego.regulatory_penalty += -2  # 确认不按规则行驶
                saved_ego.prev_laneid = current_ego.lane_id  # 更新记录的车道信息
                saved_ego.prev_roadid = current_ego.road_id  # 更新记录的路段信息
        saved_ego.st_regu += saved_ego.regulatory_penalty  # 累计
        
    def comfort(self, saved_ego: Veh_obj, current_ego: TrafficVeh):
        saved_ego.acce_recorder.append(current_ego.acc_lon)
        saved_ego.acce_lat_recorder.append(current_ego.acc_lat)

        tmp = saved_ego.acce_recorder[-1:]
        long_acce_RMS = sum([x ** 2 for x in tmp])
        tmp = saved_ego.acce_lat_recorder[-20:]
        lat_acce_RMS = sum([x ** 2 for x in tmp])

        comfort = sqrt(long_acce_RMS ** 2 + lat_acce_RMS ** 2)
        numb = 1 if comfort > 1 else 0
        
        saved_ego.st_comf.append(comfort)
        saved_ego.st_comf_lvl += numb
    
    def api_wrap(self):
        # treat traffic from sumo
        self.traffic_vehs = dict()
        for vid, rt in self.raw_traf_veh.items():
            l=rt[68]
            xy = rt[66]
            angle = rt[67]
            acc_lon = rt[114] if 114 in rt.keys() else 0
            xy, angle = sumo2dyn(l, xy, angle)
            veh = TrafficVeh(
                id=vid,
                length=l,
                width=rt[77],
                x=xy[0],
                y=xy[1],
                u=rt[64],
                acc_lon=acc_lon,
                phi=angle,
            )
            self.traffic_vehs[vid] = veh

            if vid in self.ego_name_list:
                self.ego_data[vid].update(
                    lane_index = rt[82],
                    road_id = rt[80],
                    lane_id = rt[81],
                    lane_position = rt[86],
                    lateral_position = rt[184]
                )
        
        # treat traffic light from sumo
        self.traffic_lights = dict()
        if self.raw_traf_light is not None:
            for TLid, r_tl in self.raw_traf_light.items():
                self.traffic_lights[TLid] = TrafficLight(phase=r_tl[40], state=r_tl[32])

        
def laneIDstr_resolve(laneID_str: str, get_type: str):
    """
    根据laneID获取roadID 或者 lane index，因为存在大于10车道的情况
    Args:
        laneID_str: 车道ID
        get_type: 获取类型

    Returns:
        车道ID或名称
    """
    if get_type == 'roadID':
        return laneID_str[:laneID_str.rindex('_')]
    if get_type == 'laneIndex':
        return laneID_str[laneID_str.rindex('_') + 1:]
    return ValueError
         
def sumo2dyn(l:float, xy:Tuple, angle:float) -> Tuple[Tuple[float, float], float]:
        """Coordinate transition from SUMO to LasVSim

        Args:
            l (float): length of the car
            xy (Tuple): coordinate of the car in SUMO
            angle (float): heading angle(yaw) of the car in SUMO

        Returns:
            Tuple[Tuple[float, float], float]: new coordinate and heading angle
        """
        phi = (-angle + 90) * pi / 180.
        x_dyn = xy[0] - l/2 * cos(phi)
        y_dyn = xy[1] - l/2 * sin(phi)
        return (x_dyn, y_dyn), phi

def rotation(l, w, phi):
    """phi: rad"""
    diff_x = l * np.cos(phi) - w * np.sin(phi)
    diff_y = l * np.sin(phi) + w * np.cos(phi)
    return (diff_x, diff_y)

if __name__ == '__main__':
    ...