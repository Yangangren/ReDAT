# coding=utf-8
from __future__ import print_function
from pandas import DataFrame

# ===================================================================
# save the map topology information
# author: Chen Chen
# ===================================================================

Risk_VIZ = False
Traj_VIZ = False
Data_REC = False
Other_eva = True  # TODO：是否进行其他评价指标的计算
init_figure = True

global _VClass  # vehicle class

global _RVLinfo  # road vehicle lane info某一road ID上机动车道编号列表 {road_ID:[2,3,4]}
global _RVLinfoAll  # all useful lanes
global _TMP  # a TMP test
global _RLC  # road lane count
global _R2RwDL  # road to road with direction and lenget某一road ID 能够连接到的 road ID 字典（双重字典）及方向、车道 {road_ID:{target road ID1:[dir,2,3,4],....},...}
global _RL2RaL  # road lane to road lane，某一road ID能够连接到的road ID字典（三重字典）及连接功能 {road_ID:{2:{target roadID1：2,Target roadID2：4],...},...}
global _R2RD  # roat to road distance某一road ID 能够连接到的 road ID 字典（双重字典）及距离{road_ID:{target road ID:18.25}}
global _RLPi  # road lane path index of _RP,某一road ID上的lane ID能够连接到的_road_path index{road_ID:{2:[path_index1,path_index2,..],..},..}
global _RLe  # road length，某一raod ID的长度{road_ID:length}
global _RP  # road path，某一road ID前方有可能有150m的路径列表{road_ID:[[now road,next road1,next road2...],[now road,next roadN,next roadN+1,...],...}
global _RPD  # 某一road ID前方可能有150m的路径列表对应的连接方式{road ID:[['s','l',...],...],...}
global _RListB  # Road list Basic地图基本道路ID list
global _RL2RaL_c  # road lane to road lane copy，某一road ID能够连接到的road ID字典（三重字典）及连接功能，在动态查找laneID时使用 {road_ID:{2:{target roadID1：2},2:{Target roadID2：4],...},...}

global _LAs  # road angle for straight, 某一road ID的角度{labe_DI:angle}
global _LAc  # road angle for curve, 某一road ID的角度{lane_DI:[[dis_cum, angle],[],...}
global _LPpara  # lane path parameters 根据当前lane ID确定未来路径的坐标信息{lane_ID:{n:[[(,),(,)],[(,),(,)],..],m:[],...},...}
global _LPD  # lane path direction根据当前lane ID确定未来路径的坐标信息所对应的连接方式{lane_ID:[path_index1:[s,l,...],m:[s,l,r...],...}
global _LPstpara  # lane path standard parameters 根据当前lane ID确定未来路径的'标准'坐标信息{lane_ID:{path_index1:[[d,t,r,z],[d,t,r,z],..],path_index2:[],...},...}
global _LPstparaS  # lane path standard parameters sub 相对于_LPstpara，删除了初始连接段的参数信息 {laneID:[]，...}
global _LListB  # lane list Basic地图基本道路ID list
global _LListBAll  # all lane list Basic地图基本道路ID list
global _LInfo  # lane info inclued shape, length, width, links :{laneID:{shape:f,length:f,width:f,links:[]},laneID2:{....}}
global _LCER  # lane collision edge right, 有碰撞的车道的边缘，不一定是本车道边缘 {laneID:{Shapely object Linestring}, ....}
global _LCEL  # lane collision edge right, 有碰撞的车道的边缘，不一定是本车道边缘 {laneID:{Shapely object Linestring}, ....}
global _Lshapely  # lane shapely 交集集合

global _JCEL  # junction collision edge right, 有碰撞的车道的边缘，不一定是本车道边缘 {juncID:{Shapely object Linestring}, ....}

global _VlastL  # vehicle last lane 在车辆处于连接段时使用，用于查找目标路径 {veh_name:laneID,veh_name1:laneID1,....}

global _speed_i  # 上一时刻车辆速度记录，用于重算加速度
global _speed_save  # 车速全记录
global _speed_lateral_i  # 车辆横向速度记录，用于重算速度
global _VRA  # 车辆在interval上时与各个目标roadID的夹角 {veh_name:{roadID:flaot,roadID:float,...},....}
global _VOIL  # vehicle on interval label车辆在interval上的车道选择方法标记 {veh_name:0 ro 1,.....}
global _PCHis  # path choice history 存储车辆在interval上时候的路径旋转历史信息，用于路径旋转 {}
global _PCL  # path choice label 使用初始的车道ID还是新的车道ID {veh_name:int,.....}
global _ALong  # longitude acceleration save
global _ALat  # lateral acceleration save

global _contextInfo  # 每个仿真步的周车信息集合
global _contextInof_ligth  # 每个仿真步周边信号灯信息集合

global _LS  # lane signal 车道受控的信号ID，以及第几变编号{lane_ID:[[signalID:int,signalID:int],[]],...}
global _RS  # road signal 某个路段ID是否在终端受到信号灯控制
global _BRCHECK  # break red light check  因为仅需要再过停止线的一瞬间检测是否闯红灯，所以留一个label，1表示此次过线检测过了
global _BRLI  # break red light index  是否要进行闯红灯检测

global _Risk  # risk save
global _Collided  # Collided  save
global _FC  # fuel consumption save
global _CF  # comfort save
global _TE  # Traffic efficiency save
global _RC  # regulatory complicance save
global _RunTimeI  # 运行时步
global _RunTime  # 运行时步

global _Junction_ID

global _NasEgo
global _EGOName  # 多车情况下自车集合

global _TrajREC  # 车辆轨迹记录
global _ResultsREC  # 结果记录


def _init_glo():  # 初始化
    global _VClass  # vehicle class
    _VClass = ['passenger', 'bicycle']

    global _RVLinfo  # road vehicle lane info某一road ID上机动车道编号列表 {road_ID:[2,3,4]}
    _RVLinfo = {'passenger': {}, 'bicycle': {}}
    global _RVLinfoAll  # all useful lanes
    _RVLinfoAll = {}
    global _TMP  # a TMP test
    _TMP = []
    global _RLC  # road lane count 车道数{road_ID:1,...}
    _RLC = {}
    global _R2RwDL  # road to road with direction and lenget某一road ID 能够连接到的 road ID 字典（双重字典）及方向、车道 {road_ID:{target road ID1:[dir,2,3,4],....},...}
    _R2RwDL = {'passenger': {}, 'bicycle': {}}
    global _RL2RaL  # road lane to road lane，某一road ID能够连接到的road ID字典（三重字典）及连接功能 {road_ID:{2:{target roadID1：2},2:{Target roadID2：4],...},...}
    _RL2RaL = {'passenger': {}, 'bicycle': {}}
    global _R2RD  # roat to road distance某一road ID 能够连接到的 road ID 字典（双重字典）及距离{road_ID:{target road ID:18.25}}
    _R2RD = {'passenger': {}, 'bicycle': {}}
    global _RLPi  # road lane path index of _RP,某一road ID上的lane ID能够连接到的_road_path index{road_ID:{2:[path_index1,path_index2,..],..},..}
    _RLPi = {'passenger': {}, 'bicycle': {}}
    global _RLe  # road length，某一raod ID的长度{road_ID:length}
    _RLe = {}
    global _RP  # road path，某一road ID前方有可能有150m的路径列表{road_ID:[[now road,next road1,next road2...],[now road,next roadN,next roadN+1,...],...}
    _RP = {'passenger': {}, 'bicycle': {}}
    global _RPD  # 某一road ID前方可能有150m的路径列表对应的连接方式{road ID:[['s','l',...],...],...}
    _RPD = {'passenger': {}, 'bicycle': {}}
    global _RListB  # Road list Basic地图基本道路ID list
    _RListB = []
    global _RL2RaL_c  # road lane to road lane copy，某一road ID能够连接到的road ID字典（三重字典）及连接功能，在动态查找laneID时使用 {road_ID:{2:{target roadID1：2},2:{Target roadID2：4],...},...}
    _RL2RaL_c = {'passenger': {}, 'bicycle': {}}

    global _LAs  # road angle for straight, 某一road ID的角度{labe_DI:angle}
    _LAs = {}
    global _LAc  # road angle for curve, 某一road ID的角度{lane_DI:[[dis_cum, angle],[],...}
    _LAc = {}
    global _LPpara  # lane path parameters 根据当前lane ID确定未来路径的坐标信息{lane_ID:{n:[[(,),(,)],[(,),(,)],..],m:[],...},...}
    _LPpara = {'passenger': {}, 'bicycle': {}}
    global _LPD  # lane path direction根据当前lane ID确定未来路径的坐标信息所对应的连接方式{lane_ID:[path_index1:[s,l,...],m:[s,l,r...],...}
    _LPD = {'passenger': {}, 'bicycle': {}}
    global _LPstpara  # lane path standard parameters 根据当前lane ID确定未来路径的'标准'坐标信息{lane_ID:{path_index1:[[d,t,r,z],[d,t,r,z],..],path_index2:[],...},...}
    _LPstpara = {'passenger': {}, 'bicycle': {}}
    global _LPstparaS  # lane path standard parameters sub 相对于_LPstpara，删除了初始连接段的参数信息
    _LPstparaS = {'passenger': {}, 'bicycle': {}}
    global _LListB  # lane list Basic地图基本道路ID list
    _LListB = {'passenger': [], 'bicycle': []}
    global _LListBAll  # all lane list Basic地图基本道路ID list
    _LListBAll = []
    global _LInfo  # lane info inclued shape, length, width, links :{laneID:{shape:f,length:f,width:f,links:[]},laneID2:{....}}
    _LInfo = {}
    global _LCER  # lane collision edge right, 有碰撞的车道的边缘，不一定是本车道边缘 {laneID:{Shapely object Linestring}, ....}
    _LCER = {}
    global _LCEL  # lane collision edge left, 有碰撞的车道的边缘，不一定是本车道边缘 {laneID:{Shapely object Linestring}, ....}
    _LCEL = {}
    global _Lshapely  # lane shapely 交集集合
    _Lshapely = []

    global _JCEL  # junction collision edge right, 有碰撞的车道的边缘，不一定是本车道边缘 {juncID:{Shapely object Linestring}, ....}
    _JCEL = {}

    global _VlastL  # vehicle last lane 在车辆处于连接段时使用，用于查找目标路径 {veh_name:laneID,veh_name1:laneID1,....}
    _VlastL = {}

    global _speed_i  # 车辆速度记录，用于重算速度
    _speed_i = {}
    global _speed_save  # 车速全记录
    _speed_save = []
    global _speed_lateral_i  # 车辆横向速度记录，用于重算速度
    _speed_lateral_i = {}
    global _VOIL  # 车辆在interval上的路径旋转方法标记
    _VOIL = {}
    global _VRA  # 车辆在interval上时与各个目标roadID的夹角 {veh_name:{roadID:flaot,roadID:float,...},....}
    _VRA = {}
    global _PCHis  # path choice history 存储车辆在interval上时候的路径旋转历史信息，用于路径旋转
    _PCHis = {}
    global _PCL  # path choice label 使用初始的车道ID还是新的车道ID {veh_name:int,.....}
    _PCL = {}
    global _ALong  # longitude acceleration save
    _ALong = []
    global _ALat  # lateral acceleration save
    _ALat = []

    global _contextInfo  # 每个仿真步的周车信息集合
    _contextInfo = []
    global _contextInof_ligth  # 每个仿真步周边信号灯信息集合
    _contextInof_ligth = []

    global _LS  # lane signal 车道受控的信号ID，以及第几变编号{lane_ID:[[signalID,int],[signalID,int],...],...}
    _LS = {}
    global _RS  # road signal 某个路段ID是否在终端受到信号灯控制 [roadID,]
    _RS = []
    global _BRCHECK  # break red light check  因为仅需要再过停止线的一瞬间检测是否闯红灯，所以留一个label，1表示此次过线检测过了
    _BRCHECK = 0
    global _BRLI  # break red light index  是否要进行闯红灯检测
    _BRLI = []  # 0表示开始就没有闯红灯，不再做后续两步判断

    global _Risk  # risk save
    _Risk = []
    global _Collided  # risk save
    _Collided = []
    global _FC  # fuel consumption save
    _FC = []
    global _CF  # comfort save
    _CF = []
    global _TE  # Traffic efficiency save
    _TE = []
    global _RC  # regulatory complicance save
    _RC = {}
    global _RunTimeI  # 运行时步
    _RunTimeI = 0
    global _RunTime  # 运行时步
    _RunTime = []

    global _Junction_ID
    _Junction_ID = 0

    global _NasEgo
    _NasEgo = 'ego'
    global _EGOName  # 多车情况下自车集合
    _EGOName = []

    global _TrajREC  # 车辆轨迹记录
    _TrajREC = DataFrame()
    global _ResultsREC  # 结果记录
    _ResultsREC = DataFrame()

    # print('[DATA_STRUCTURES MODULE]: Map topology data structure has been initialized.')

_init_glo()

if __name__ == "__main__":
    pass
