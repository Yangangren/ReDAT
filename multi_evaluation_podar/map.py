# This Python file uses the following encoding: utf-8
# coding=utf-8

# ===================================================================
# get the map topology information to support the navigation and evaluation module.
# author: Chen Chen
# ===================================================================

from traci import lane, trafficlight, edge, junction
from shapely.geometry import LineString, Polygon
from math import *
from collections import deque
import numpy as np
import time
from multi_evaluation_podar import map_topo

class Map(object):
    def __init__(self):
        pass

    def update(self):
        """
        [update method to sequentially establish map topology]
        """
        tic = time.time()
        print('[MAP]: Process map information...')
        print('[MAP]: - Build edge and lane topology...', end='')
        Road_pre_cal().use()
        # Road_info_cal().use()
        Lane_info_cal()
        traffic_control_map()
        print('Done')

        # print('[MAP]: - Build edge collision entity...', end='')
        # lane_edge_cal()
        # print('Done')

        print('[MAP]: DONE (module running time={:.3f}s)'.format((time.time() - tic)))


class Road_pre_cal(object):
    """
    get information of all the sumo roads.
    """
    def __init__(self):
        self.road_ID_list = edge.getIDList()

    def vehicle_lane_num_count(self, road_ID):
        """
        [count the number of lanes of the road, and save it to global_vars]
        Args:
            road_ID: sumo road ID
        """
        c_1 = map_topo._VClass[0]
        c_2 = map_topo._VClass[1]
        VLinfo = list()
        lane_ID_base = list()  # 用于存储机动车道基本laneID的列表

        VLinfo_bic = list()  # 自行车的信息列表
        lane_ID_base_bic = list()

        N = edge.getLaneNumber(road_ID)
        map_topo._RLC[road_ID] = N  # 记录每个道路的车道数
        for i in range(N):
            now_laneID = ''.join([road_ID, '_', str(i)])
            allow_type = lane.getAllowed(now_laneID)
            disallow_type = lane.getDisallowed(now_laneID)
            if ('bicycle',) == allow_type:
                VLinfo_bic.append(i)
                lane_ID_base_bic.append(now_laneID)
            elif (not 'passenger' in disallow_type) or ('bus' in allow_type):
                VLinfo.append(i)
                lane_ID_base.append(now_laneID)

            if allow_type != (): map_topo._TMP.append(now_laneID)

        if len(VLinfo_bic) == 0 and len(VLinfo) != 0:  # 没有自行车道
            VLinfo_bic = [min(VLinfo)]  # 那么最右侧的一条机动车道作为自行车道
            lane_ID_base_bic = [lane_ID_base[0]]
            for i in VLinfo[1:]:
                laneID = ''.join([road_ID, '_', str(i)])
                lane.setDisallowed(laneID, ['pedestrian', 'bicycle'])
        else:
            for i in VLinfo:
                laneID = ''.join([road_ID, '_', str(i)])
                lane.setDisallowed(laneID, ['pedestrian', 'bicycle'])

        map_topo._RVLinfo[c_1][road_ID] = VLinfo
        map_topo._LListB[c_1] = map_topo._LListB[c_1] + lane_ID_base

        map_topo._RVLinfo[c_2][road_ID] = VLinfo_bic
        map_topo._LListB[c_2] = map_topo._LListB[c_2] + lane_ID_base_bic

        map_topo._RVLinfoAll[road_ID] = list(set(VLinfo + VLinfo_bic))

    def road_connection(self):
        """
        [计算与某个road_ID相连接的road ID字典和平均距离字典]
        """
        for vehicle_class in map_topo._VClass:
            target_road = {}
            target_lane = {}
            target_length = {}
            l_label = 0
            r_label = 0
            VLinfo = map_topo._RVLinfo[vehicle_class][self.road_ID][:]
            VLinfo.reverse()
            for i in VLinfo:  # 循环当前道路各个车道-从大到小循环，在机动车道编号列表[2,3,5,..]中循环
                lane_ID = self.road_ID + '_' + str(i)  # 车道编号 str
                tmp = lane.getLinks(lane_ID)  # 某条车道的连接信息
                target_lane[i] = {}  # 初始化子字典
                same_target_rID_l = []  # 这个车道右转连接到的roadID的集合
                same_target_rID_s = []  # 这个车道直行连接到的roadID的集合
                for j in range(len(tmp)):  # 对于每个目标road ID，在连接数量range(n)中循环
                    if tmp[j][0][0] == ':': continue  # 在清华地图中，出现了link列表中存在非基本路段的情况，在这里进行验证
                    if tmp[j][0] not in map_topo._LListB[vehicle_class]: continue
                    tmp_road_ID = laneIDstr_resolve(tmp[j][0],
                                                    get_type='roadID')  # 理论上讲，进入某一段路选择任何一个车道都可以，所以这里直接取road_ID
                    tmp_lane_function = tmp[j][-2]  # 与当前车道的关系，即方向
                    # 左转情况，且同一车道对同一路段只要有1次左转，就设置这个标记，下几次的左转都不管了
                    if (tmp_lane_function in 'ltL') and (not tmp_road_ID in same_target_rID_l):
                        veh_lane = map_topo._RVLinfo[vehicle_class][tmp_road_ID][:]
                        if len(veh_lane) == 0: continue
                        veh_lane.reverse()
                        x1, y1 = lane.getShape(self.road_ID + '_' + str(i))[-1]
                        if l_label <= len(veh_lane) - 1:
                            tmp_t_lane = veh_lane[l_label]
                        else:
                            tmp_t_lane = veh_lane[-1]
                        x2, y2 = lane.getShape(tmp_road_ID + '_' + str(tmp_t_lane))[0]
                        d1 = distance_cal(x1, y1, x2, y2)
                        tmp_connection_length = (abs(x2 - x1) + abs(y2 - y1) + d1 * 0.95) / 2  # 一个约数，作为实际距离
                        same_target_rID_l.append(tmp_road_ID)
                    # 直行情况,且同一车道对同一路段只要有1次直行，就设置这个标记，下几次的左转都不管了
                    elif (tmp_lane_function == 's') and (not tmp_road_ID in same_target_rID_s):
                        d = 8192
                        veh_lane = map_topo._RVLinfo[vehicle_class][tmp_road_ID][:]
                        if len(veh_lane) == 0: continue
                        x1, y1 = lane.getShape(self.road_ID + '_' + str(i))[-1]
                        for k in veh_lane:  # 查找最大可能的目标车道
                            x2, y2 = lane.getShape(tmp_road_ID + '_' + str(k))[0]
                            d1 = distance_cal(x1, y1, x2, y2)
                            if d1 < d:
                                tmp_t_lane = k
                                tmp_connection_length = d1
                                d = d1
                        same_target_rID_s.append(tmp_road_ID)
                    else:
                        continue
                    # 以road ID为键值的 以target lane ID为键值 的存储
                    target_lane[i][tmp_road_ID] = tmp_t_lane
                    # 以目标道路ID为键值的存储
                    if not tmp_road_ID in target_road:  # 如果局部连接信息中还没有这个目标road ID
                        target_road[tmp_road_ID] = list([tmp_lane_function, i])  # 新建key将能够去往目标road ID的车道编号放入字典
                        target_length[tmp_road_ID] = tmp_connection_length  # 新建key 放入距离
                    else:  # 如果已经存在，
                        target_road[tmp_road_ID].append(i)  # 将车道编号附加到后边
                        target_length[tmp_road_ID] = (target_length[tmp_road_ID] + tmp_connection_length) / 2  # 求均值
                    l_label += 1 if tmp_lane_function == 'l' else l_label  # 检查是否有多个左转或右转车道

            VLinfo.reverse()
            for i in VLinfo:  # 循环当前道路各个车道-从大到小循环，在机动车道编号列表[2,3,5,..]中循环
                lane_ID = self.road_ID + '_' + str(i)  # 车道编号 str
                tmp = lane.getLinks(lane_ID)  # 某条车道的连接信息
                if not i in target_lane.keys():
                    target_lane[i] = {}  # 初始化子字典
                same_target_rID_r = []  # 这个车道右转连接到的roadID的集合
                for j in range(len(tmp)):  # 对于每个目标road ID，在连接数量range(n)中循环
                    if tmp[j][0][0] == ':': continue  # 在清华地图中，出现了link列表中存在非基本路段的情况，在这里进行验证
                    if tmp[j][0] not in map_topo._LListB[vehicle_class]: continue
                    tmp_road_ID = laneIDstr_resolve(tmp[j][0],
                                                    get_type='roadID')  # 理论上讲，进入某一段路选择任何一个车道都可以，所以这里直接取road_ID
                    tmp_lane_function = tmp[j][-2]  # 与当前车道的关系，即方向
                    # 右转情况
                    if (tmp_lane_function in 'rR') and (not tmp_road_ID in same_target_rID_r):
                        veh_lane = map_topo._RVLinfo[vehicle_class][tmp_road_ID][:]
                        if len(veh_lane) == 0: continue
                        x1, y1 = lane.getShape(self.road_ID + '_' + str(i))[-1]
                        if r_label <= len(veh_lane) - 1:
                            tmp_t_lane = veh_lane[r_label]
                        else:
                            tmp_t_lane = veh_lane[-1]
                        x2, y2 = lane.getShape(tmp_road_ID + '_' + str(tmp_t_lane))[0]
                        d1 = distance_cal(x1, y1, x2, y2)
                        tmp_connection_length = (abs(x2 - x1) + abs(y2 - y1) + d1 * 0.95) / 2  # 一个约数，作为实际距离
                        same_target_rID_r.append(tmp_road_ID)
                    else:
                        continue
                    # 以road ID为键值的 以target lane ID为键值 的存储
                    target_lane[i][tmp_road_ID] = tmp_t_lane
                    # 以目标道路ID为键值的存储
                    if not tmp_road_ID in target_road:  # 如果局部连接信息中还没有这个目标road ID
                        target_road[tmp_road_ID] = list([tmp_lane_function, i])  # 新建key将能够去往目标road ID的车道编号放入字典
                        target_length[tmp_road_ID] = tmp_connection_length  # 新建key 放入距离
                    else:  # 如果已经存在，
                        target_road[tmp_road_ID].append(i)  # 将车道编号附加到后边
                        target_length[tmp_road_ID] = (target_length[tmp_road_ID] + tmp_connection_length) / 2  # 求均值
                    r_label += 1 if tmp_lane_function == 'r' else r_label

            map_topo._R2RwDL[vehicle_class][self.road_ID] = target_road  # 结束后将信息更新到全局变量中
            map_topo._RL2RaL[vehicle_class][self.road_ID] = target_lane
            map_topo._R2RD[vehicle_class][self.road_ID] = target_length

    def use(self):
        """
        运行此类
        """
        road_ID_base = []
        for road_ID in self.road_ID_list:  # 这里可以改用map
            if road_ID[0] != ':':
                self.vehicle_lane_num_count(road_ID)
                road_ID_base.append(road_ID)
        map_topo._RListB = road_ID_base
        # map_topo._LListBAll = list(
        #     set(map_topo._LListB[map_topo._VClass[0]] + map_topo._LListB[map_topo._VClass[1]]))
        for i_all in road_ID_base:  # 这里可以改用map
            self.road_ID = i_all
            self.road_connection()
            map_topo._RLe[self.road_ID] = lane.getLength(self.road_ID + '_0')
        map_topo._RL2RaL_c = map_topo._RL2RaL  # 复制这一全局变量用于后边动态查找目标车道id


# 关于道路的计算类，包括机动车道、可连接道路数量及对应车道、距离的计算
class Road_info_cal(object):
    def __init__(self):
        pass

    def depth_first(self):
        """使用深度优先搜索来搜索当前所处road ID能够在150m范围内连接到的道路"""
        for vehicle_class in map_topo._VClass:
            search_queue = deque()  # 初始化搜索集
            search_queue.append(self.road_ID)  # 初始化搜索集-加入起始点
            visited = []  # 访问集
            self.path = []  # 完整road ID path路径集
            self.path_dir = []  # 完整road ID path路径集对应的连接方式
            self.pathtmp = []  # tmp road ID path路径集

            person = self.road_ID
            while search_queue:  # 若搜索集中还有信息
                if person in visited:  # 如果当前节点已经搜索过，即树弹回上一级，则将路径序列中上一子根节点剪除
                    self.pathtmp.pop(-1)
                else:  # 执行向下的树
                    self.pathtmp.append(person)  # 将当前节点加入到临时的path中
                    self.now_dis = 0  # 这里没有使用rest_dis 是因为想要放到一个全局变量中，这样其他车辆就不用再搜寻一次
                    for k in range(len(self.pathtmp) - 1):  # 计算当前路径是否已经满足条件
                        pre = self.pathtmp[k]
                        next = self.pathtmp[k + 1]
                        self.now_dis += map_topo._R2RD[vehicle_class][pre][next] + map_topo._RLe[next]
                    if (self.now_dis < 150) and (person in map_topo._R2RD[vehicle_class].keys()) and (
                            len(map_topo._R2RD[vehicle_class][person]) != 0):  # 截止条件：距离超过150m TODO：未到150就没路了
                        search_queue.appendleft(person)  # 将person放到搜索集中，用于之后检验是否从树杈上跳出
                        tmp = map_topo._R2RD[vehicle_class][person]  # 提取子节点person的信息
                        for index in tmp.keys():  # 将子节点person中的子节点key信息加到搜索序列
                            search_queue.appendleft(index)
                        visited.append(person)  # 将已经搜索过的非底层节点加到访问集中
                    elif person != self.road_ID:  # 如果当前路径已经超过150m距离
                        tmp_dir = []
                        for j in range(len(self.pathtmp) - 1):
                            pre = self.pathtmp[j]
                            next = self.pathtmp[j + 1]
                            tmp_dir.append(map_topo._R2RwDL[vehicle_class][pre][next][0])  # 存储方向
                        self.path.append(self.pathtmp[:])
                        self.path_dir.append(tmp_dir)
                        self.pathtmp.pop(-1)

                person = search_queue.popleft()
            map_topo._RP[vehicle_class][self.road_ID] = self.path  # 将搜索到的路径写入到全局变量方便下次使用
            map_topo._RPD[vehicle_class][self.road_ID] = self.path_dir  # 连接方向

    def lane_to_road(self):
        """根据当前道路的每个车道能连接到的道路ID，提取可能的path表"""
        for vehicle_class in map_topo._VClass:
            tmp_var = {}
            for i in map_topo._RVLinfo[vehicle_class][self.road_ID]:  # 循环各个机动车道 i = int
                k = 0
                tmp_con = map_topo._RL2RaL[vehicle_class][self.road_ID][
                    i]  # 当前道路车道连接信息i=[target roadID1:lane,target roadID2:lane]
                for j in map_topo._RP[vehicle_class][
                    self.road_ID]:  # 循环各个路径j=[now road,next road1,next road2...]
                    if j[1] in tmp_con:  # 如果该车道能连接到这个路径
                        try:
                            tmp_var[i].append(k)
                        except:
                            tmp_var[i] = list([k])
                    k += 1
            map_topo._RLPi[vehicle_class][self.road_ID] = tmp_var

    def use(self):
        """使用这个函数的入口"""
        for i_all in map_topo._RListB:  # 这里可以改用map
            self.road_ID = i_all
            self.depth_first()
            self.lane_to_road()

# 车道级计算，包括车道之间的连接关系等，并最终转换为标准数据结构保存到全局变量
class Lane_info_cal(object):
    def __init__(self):
        self.lane_ID_list = lane.getIDList()

        for now_laneID in self.lane_ID_list:
            # 预获取车道信息
            lane_info = {}
            lane_info['shape'] = lane.getShape(now_laneID)
            lane_info['links'] = lane.getLinks(now_laneID)
            lane_info['width'] = lane.getWidth(now_laneID)
            add_length = 10 if len(lane_info['links']) == 0 else 0  # 如果后续没有连接段，也就是地图边界了：将当前道路长度延长10米避免报错
            lane_info['length'] = lane.getLength(now_laneID) + add_length

            # 存储车道信息
            map_topo._LInfo[now_laneID] = lane_info

    def path_choice(self):
        """根据全局变量中存储的路径信息，计算当前所处车道未来可能的路径，具体到坐标点或轨迹参数点"""
        path_list = map_topo._RP[self.vehicle_class][self.road_ID]  # 可能的路径列表
        path_dir_list = map_topo._RPD[self.vehicle_class][self.road_ID]  # 可能的路径列表对应的连接方式
        posible_path = map_topo._RLPi[self.vehicle_class][self.road_ID][self.lane_this_end]  # 目标车道可能的路径index
        self.path_para = {}  # 路径参数点集合
        self.path_dir = {}  # 路径方向集合
        for i in posible_path:  # 循环每个可能路径,i=int
            tmp_path = path_list[i]  # tmp_path = [now road,next road1,next road2...]
            tmp_path_dir = path_dir_list[i]
            tmp_path_para = [(lane.getShape(self.lane_ID)[-2], lane.getShape(self.lane_ID)[-1])]  # 第一行时当前所处车道的最后2组坐标
            pre_lane = self.lane_this_end  # int
            for j in range(len(tmp_path[1:])):  # 循环每个lane提取关键参数点坐标，除去本道路
                tmp_label = 0
                if tmp_path[j + 1] in map_topo._RL2RaL[self.vehicle_class][tmp_path[j]][
                    pre_lane]:  # 这里判定是因为存在在下一段路上换道情况
                    tmp_label = 1
                    next_lane = map_topo._RL2RaL[self.vehicle_class][tmp_path[j]][pre_lane][
                        tmp_path[j + 1]]  # int
                    tmp_path_para.append(lane.getShape(tmp_path[j + 1] + '_' + str(next_lane)))  # 附加第j个目标车道的坐标点
                    pre_lane = next_lane
                else:
                    break
            if tmp_label == 1:
                self.path_para[i] = tmp_path_para
                self.path_dir[i] = tmp_path_dir
        map_topo._LPpara[self.vehicle_class][self.save_lane_index] = self.path_para
        map_topo._LPD[self.vehicle_class][self.save_lane_index] = self.path_dir

    def lane_path_standard_para(self):
        """
        将获得的路径信息，转换为用于轨迹预测旋转的标准结构体：
        [距离当前位置距离, 路段类别（1=直，3=圆）, 圆半径, 转弯方向（0=左，1=右]
        [[0,  0,  0,  0],
         [d1, t1, r1, p1],
         ...]
        """
        virtual_dis = 1.5  # 虚拟的用于折线路段的切线值
        LPpara_set = {}  # 初始化多个lane path 的字典
        LPpara_set_1 = {}
        for i_path in map_topo._LPpara[self.vehicle_class][self.save_lane_index].keys():  # 循环可能的路径数
            LPstpara = [[0, 0, 0, 0]]  # 标准化后的路径参数集合初始化。这里第一行赋0是因为后边连接段会增加或减少第一段距离
            dis_cum = 0  # 第一列距离数值初始化
            LPpara = map_topo._LPpara[self.vehicle_class][self.save_lane_index][i_path]  # 当前路径的参数集合list
            LPDir = map_topo._LPD[self.vehicle_class][self.save_lane_index][i_path]  # 当前路径的连接方式（方向）集合list
            # 注意，LPDir的长度是n，LPpara长度是n+1，含有当前车道的终点组参数
            n = len(LPDir)  # 当前路径的道路分段数
            i_road = 0
            while i_road < n:  # 这里使用while是考虑将近距离所有连续直行的道路合并考虑
                # 先考虑连接部分，以下两个二选一
                if LPDir[i_road] == 's':  # 如果为直线连接方式
                    para_pre = length_minus(LPpara[i_road][-2:], virtual_dis)  # 构造用于处理连接段的前一路段坐标
                    j = i_road
                    while (len(LPpara[j + 1]) == 2) and (j + 1 < n):  # 检查是否合并直线路段，阈值为3m以内的路段
                        d_tmp = distance_cal(LPpara[j + 1][0][0], LPpara[j + 1][0][1],
                                             LPpara[j + 1][1][0], LPpara[j + 1][1][1])  # 下一直线路段的长度
                        if (LPDir[j + 1] != 's') or (d_tmp > 3): break  # 如果下一连接方式不是直线连接了或长度较长，break
                        j += 1
                    i_road = j  # 将i_road更新为合并之后的j
                    para_next = list(LPpara[i_road + 1][:2])  # 构造用于处理连接段的后一路段坐标
                    tmp_para = para_pre + para_next  # 视为弯曲路段的参数集构造： 4个点3段
                    scpara_tmp = curve_road_treat(tmp_para, dis_cum, virtual_dis)  # 视为弯曲路段来计算路径参数
                    LPstpara = LPstpara + scpara_tmp[:-1]  # 附到总路径中，这里使用range[:-1]是因为只提取了圆-直-圆的3段
                    dis_cum = LPstpara[-1][0]  # 更新累积距离，注意，这里因为最后一个圆吃掉了后边直线的virtual_dis距离，后边要处理
                else:  # 如果为左转或右转连接方式
                    coor_pre = LPpara[i_road][-2:]  # 构造参数
                    coor_next = LPpara[i_road + 1][:2]
                    # LPstpara[-1][0] -= virtual_dis  # 减去3m，以和直线路段的构造一致
                    ccpara, d_left, d_label = curve_cross_cal(coor_pre, coor_next, dis_cum)  # 计算圆曲线参数
                    ccpara[0] = ccpara[0] + virtual_dis
                    if d_label == 0:
                        para_t = [LPstpara[-1][0] + d_left + virtual_dis, 1, 0, 0]  # 前边的直线段，类似直线连接，补回开头占用
                        LPstpara.append(para_t)
                        LPstpara.append(ccpara)  # 中间的圆段
                        LPstpara.append([LPstpara[-1][0] + virtual_dis, 1, 0, 0])  # 后边的直线段——与直线连接相同构造，占后边直线3m
                    else:
                        LPstpara.append([LPstpara[-1][0] + virtual_dis, 1, 0, 0])  # 前边的直线段，类似直线连接，补回开头占用
                        LPstpara.append(ccpara)  # 中间的圆段
                        para_t = [ccpara[0] + d_label + virtual_dis, 1, 0, 0]  # 后边的直线段——与直线连接相同构造，占后边直线3m
                        LPstpara.append(para_t)
                    dis_cum = LPstpara[-1][0]  # 更新累积距离，吃掉virtual_dis距离才对。后边路径要加回来

                # 再考虑后边路径
                if len(LPpara[i_road + 1]) > 2:  # 处理不是直线的路段
                    stpara_tmp = curve_road_treat(LPpara[i_road + 1], dis_cum - virtual_dis, virtual_dis)
                    LPstpara = LPstpara + stpara_tmp  # 附加当前路径到总路径中
                    dis_cum = LPstpara[-1][0]  # 更新累积距离
                else:  # 处理是直线的路段
                    dis_s = distance_cal(LPpara[i_road + 1][0][0], LPpara[i_road + 1][0][1],
                                         LPpara[i_road + 1][1][0], LPpara[i_road + 1][1][1])  # 计算距离
                    LPstpara.append([dis_cum + dis_s - virtual_dis * 2, 1, 0, 0])  # 这里减virtual_dis*2是因为连接段构造前后占3m
                    dis_cum = LPstpara[-1][0]  # 更新累积距离
                i_road += 1  # 更新迭代road变量
            LPstpara_s = LPstpara[:]
            LPstpara_s = repeat_delete(LPstpara_s)
            LPpara_set[i_path] = LPstpara_s[1:]  # 保存当前路径数据
            if LPDir[0] == 's':  # 因为当车辆在interval段上时，第一连接段的信息需要重新计算，因此需要删除
                tmp_v = LPstpara[4][0]
                LPstpara = LPstpara[5:]  # 第一连接段是直线时，删除前四段（起-圆-直-圆段）
                LPstpara = list(map(lambda q: [q[0] - tmp_v + virtual_dis] + q[1:], LPstpara))
            else:
                tmp_v = LPstpara[3][0]
                LPstpara = LPstpara[4:]  # 第一连接段时转弯时，删除前四段（起-直-圆-直段）
                LPstpara = list(map(lambda q: [q[0] - tmp_v + virtual_dis] + q[1:], LPstpara))
            LPstpara = repeat_delete(LPstpara)
            LPpara_set_1[i_path] = LPstpara[:]
        map_topo._LPstpara[self.vehicle_class][self.save_lane_index] = LPpara_set  # 写入全局变量
        map_topo._LPstparaS[self.vehicle_class][self.save_lane_index] = LPpara_set_1

    def lane_angle_cal(self):
        """计算道路角度'"""
        self.laneshape = lane.getShape(self.lane_ID)
        if len(self.laneshape) == 2:  # 直线路段
            x1, y1, x2, y2 = self.laneshape[0][0], self.laneshape[0][1], self.laneshape[1][0], self.laneshape[1][1]
            if x2 - x1 == 0:
                self.lane_angle = pi / 2.0 if y1 - y2 < 0 else -pi / 2.0
            elif y2 - y1 == 0:
                self.lane_angle = 0 if x1 - x2 < 0 else pi
            else:
                self.lane_angle = atan2((y2 - y1), (x2 - x1))  # 输出[-pi.pi]
            # self.lane_angle = 2 * pi + self.lane_angle if self.lane_angle < 0 else self.lane_angle  # 输出[0,2pi]
            map_topo._LAs[self.save_lane_index] = self.lane_angle
        else:  # 弯曲路段
            dis_cum = 0
            self.A_record = []
            for i in range(len(self.laneshape) - 1):
                x1, y1, x2, y2 = self.laneshape[i][0], self.laneshape[i][1], self.laneshape[i + 1][0], \
                                 self.laneshape[i + 1][1]
                if x2 - x1 == 0:
                    angle_tmp = pi / 2.0 if y1 - y2 < 0 else -pi / 2.0
                elif y2 - y1 == 0:
                    angle_tmp = 0 if x1 - x2 < 0 else pi
                else:
                    angle_tmp = atan2((y2 - y1), (x2 - x1))
                # angle_tmp = 2 * pi + angle_tmp if angle_tmp < 0 else angle_tmp  # 输出[0,2pi]
                d_tmp = distance_cal(x1, y1, x2, y2)
                dis_cum += d_tmp
                self.A_record.append([dis_cum, angle_tmp])
            map_topo._LAc[self.save_lane_index] = self.A_record

    def use(self):
        for vehicle_class in map_topo._VClass:
            self.vehicle_class = vehicle_class
            for i_all in map_topo._LListB[vehicle_class]:
                self.lane_ID = i_all
                self.road_ID = laneIDstr_resolve(self.lane_ID, get_type='roadID')  # 当前所在的road ID

                self.lane_this_end = int(laneIDstr_resolve(self.lane_ID, get_type='laneIndex'))  # 出口车道，int
                self.save_lane_index = self.lane_ID
                self.lane_angle_cal()

                if not len(map_topo._RP[vehicle_class][self.road_ID]) == 0:  # 如果后边有连接段，能搜索
                    self.path_choice()
                    self.lane_path_standard_para()
                else:  # 没有的话，也需要赋值，否则后边调用的时候出错
                    map_topo._LPstpara[vehicle_class][self.save_lane_index] = {0: [[10000, 1, 0, 0]]}  # 写入全局变量
                    map_topo._LPstparaS[vehicle_class][self.save_lane_index] = {0: [[10000, 1, 0, 0]]}

# 构建车道边缘信息，并转换为shapely对象用于评价
class lane_edge_cal(object):
    def __init__(self):
        self.static_edge_para()
        self.static_junction_para()
        self.edge_concat()

    def static_edge_para(self):
        """
        获取车道边缘信息并转换为shapely对象
        """
        i = 0
        for lane_ID in map_topo._TMP:
            shape_l = lane.getShape(lane_ID)
            if i == 0:
                self.min_x, self.min_y = np.array(shape_l).min(0)
                self.max_x, self.max_y = np.array(shape_l).max(0)
                i += 1
            else:
                self.min_x, self.min_y = min([self.min_x, np.array(shape_l).min(0)[0]]), \
                                         min([self.min_y, np.array(shape_l).min(0)[1]])
                self.max_x, self.max_y = max([self.max_x, np.array(shape_l).max(0)[0]]), \
                                         max([self.max_y, np.array(shape_l).max(0)[1]])
            width = lane.getWidth(lane_ID) / 2 + 0.01
            map_topo._LCEL[lane_ID] = LineString(shape_l).buffer(width, cap_style=3)

        for road_ID in map_topo._RListB:
            N = map_topo._RLC[road_ID]  # 总的车道数量
            lane_ID = road_ID + '_' + str(N - 1)
            shape_l = lane.getShape(lane_ID)
            width = lane.getWidth(lane_ID) / 2
            map_topo._LCER[road_ID] = LineString(shape_l).parallel_offset(width, 'left', join_style=3,
                                                                                 mitre_limit=30).buffer(0.01)

    def static_junction_para(self):
        """
        获取junction信息并保存
        """
        aj = junction.getIDList()
        for junc_ID in aj:
            if junc_ID[0] == ':': continue
            shape_j = junction.getShape(junc_ID)
            if len(shape_j) >= 4:
                map_topo._JCEL[junc_ID] = shape_j

    def edge_concat(self):
        """
        是要那个shapely中方法计算非行驶区域并转换为shapely对象
        """
        m = Polygon()
        for i, info in map_topo._LCEL.items():
            m = m.union(info)
        for i, info in map_topo._JCEL.items():
            k = Polygon(info)
            m = m.union(k)
        for i, info in map_topo._LCER.items():
            m = m.difference(info)

        max_m = Polygon(((self.min_x - 100, self.min_y - 100),
                         (self.min_x - 100, self.max_y + 100),
                         (self.max_x + 100, self.max_y + 100),
                         (self.max_x + 100, self.min_y - 100)))

        map_topo._Lshapely = max_m.difference(m)


def curve_cross_cal(coor_pre, coor_next, distance):
    '''
    根据前一路段的最后两个坐标点和后一路段的前两个坐标点计算直线交点
    Args:
        coor_pre(list): 前一段的坐标点
        coor_next(list): 后一段的坐标点
        distance(float): 已有的路径长度
    Returns:
        list:[R, L, z, d_left, label]
        R: 半径
        L: 弧长
        z: 转向标记，左转0，右转1
        d_left: 剩余直线距离
        label: 剩余长度加到哪个直线，0为前一路段，1为后一路段，-1为相等
    '''
    x11, y11, x12, y12 = coor_pre[0][0], coor_pre[0][1], coor_pre[1][0], coor_pre[1][1]
    x21, y21, x22, y22 = coor_next[0][0], coor_next[0][1], coor_next[1][0], coor_next[1][1]
    A1, B1, C1 = y12 - y11, x11 - x12, y11 * x12 - x11 * y12
    A2, B2, C2 = y22 - y21, x21 - x22, y21 * x22 - x21 * y22
    label = 0
    # if x12 - x11 == 0:
    #     X = x11
    #     if B2 != 0:
    #         Y = -C2 / B2
    #     else:
    #         label = 1
    # elif y12 - y11 == 0:
    #     if A2 != 0:
    #         X = -C2 / A2
    #     else:
    #         label = 1
    #     Y = y11
    #
    # if x22 - x21 == 0:
    #     X = x21
    #     if B1 != 0:
    #         Y = -C1 / B1
    #     else:
    #         label = 1
    # elif y22 - y21 == 0:
    #     if A1 != 0:
    #         X = -C1 / A1
    #     else:
    #         label = 1
    #     Y = y21

    if A1 * B2 - A2 * B1 != 0:  # 不共线
        X = (B1 * C2 - B2 * C1) / (A1 * B2 - A2 * B1)  # 交点坐标
        Y = (C1 * A2 - C2 * A1) / (A1 * B2 - A2 * B1)
    else:
        label = 1  # 这种情况是共线的情况，在转弯处，只可能是掉头，所以角度就是180度

    if label == 0:
        xx1, yy1, xx2, yy2 = x12 - x11, y12 - y11, x22 - x21, y22 - y21
        a1 = sqrt(xx1 ** 2 + yy1 ** 2)
        a2 = sqrt(xx2 ** 2 + yy2 ** 2)
        thi = acos(np.clip((xx1 * xx2 + yy1 * yy2) / (a1 * a2), -1., 1.))  # 方向夹角
        ro_label = xx1 * yy2 - xx2 * yy1

        d1 = distance_cal(x12, y12, X, Y)  # 前一车道坐标点距离交叉点距离
        d2 = distance_cal(x21, y21, X, Y)  # 前一车道坐标点距离交叉点距离

        d = min(d1, d2)  # 切线长
        d_left = max(d1, d2) - d  # 剩余长度，加到前边或后一段直线上
        label = -1 if d1 == d2 else 1 if min(d1, d2) == d1 else 0  # 如果最小值为d1，返回1，将d_left加入到后一路段，否则0,加入到前一路段，相等返回-1
        alf = abs(thi)  # 夹角
        R = d / abs(tan(alf / 2))  # 半径
        L = R * alf  # 弧长
        z = 0 if ro_label > 0 else 1
        dis = distance + L if label == 1 else distance + L + d_left
    else:
        thi = pi
        dd = distance_cal(x12, y12, x21, y21)
        R = dd / 2
        L = R * thi
        z = 0
        dis = distance + L
        d_left, label = 0, 0

    return [dis, 3, R, z], d_left, label


def curve_road_treat(coor, distance, virtual_dis):
    '''
    处理不是直线路段的情况，将连接段处视为一个小的转弯
    Args:
        coor(list): 车道参数点的集合
        distance(float): 已有的路径长度
    Returns:
        var_rp(list): 标准参数
    '''
    path_para = []  # 构造参数集合初始化
    dis_cum = distance + virtual_dis  # 因为第一段只减去切线长3，而后边中间段为减6，这里补回来
    for i in range(len(coor) - 2):
        d = virtual_dis  # 3米的转弯切线长
        x11, y11, x12, y12 = coor[i][0], coor[i][1], coor[i + 1][0], coor[i + 1][1]
        x21, y21, x22, y22 = coor[i + 1][0], coor[i + 1][1], coor[i + 2][0], coor[i + 2][1]
        d_pre = distance_cal(x11, y11, x12, y12) - virtual_dis * 2  # 交点前的路径长度

        xx1, yy1, xx2, yy2 = x12 - x11, y12 - y11, x22 - x21, y22 - y21
        a1 = sqrt(xx1 ** 2 + yy1 ** 2)
        a2 = sqrt(xx2 ** 2 + yy2 ** 2)
        tmp_value = (xx1 * xx2 + yy1 * yy2) / (a1 * a2)
        thi = acos(tmp_value)  # 方向夹角

        if (thi == 0):  # 如果真的没有夹角，而且对齐，就按直线处理
            dis_cum += d_pre  # 这里分类弄是因为后边的处理中需要返回来删除interval上的数据，所以分开比较好处理
            path_para.append([dis_cum, 1, 0, 0])  # 累加
            dis_cum += virtual_dis * 2
            path_para.append([dis_cum, 1, 0, 0])  # 累加
            continue

        R = d / abs(tan(thi / 2.0))  # 半径
        L = R * thi  # 弧长
        z = 0 if ((xx1 * yy2 - yy1 * xx2) > 0) else 1  # 转弯方向标记，0左1右
        dis_cum += d_pre  # 累积路径距离-上一直线段
        path_para.append([dis_cum, 1, 0, 0])  # 累加
        dis_cum += L  # 累积路径距离-圆曲线段
        path_para.append([dis_cum, 3, R, z])
    d_end = distance_cal(x21, y21, x22, y22) - virtual_dis  # 最后一段直线段长度
    dis_cum += d_end  # 累积最后一段直线路径距离
    path_para.append([dis_cum, 1, 0, 0])

    return path_para  # [[],[]]


def distance_cal(x1, y1, x2, y2):
    # type: (float, float, float, float) -> float
    """calculate the distance between two point"""
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))


def laneIDstr_resolve(laneID_str, get_type):
    '''根据laneID获取roadID 或者 lane index，因为存在大于10车道的情况'''
    if get_type == 'roadID':
        return laneID_str[:laneID_str.rindex('_')]
    if get_type == 'laneIndex':
        return laneID_str[laneID_str.rindex('_') + 1:]
    if get_type == 'all':
        return [laneID_str[:laneID_str.rindex('_')], laneID_str[laneID_str.rindex('_') + 1:]]
    return ValueError


def length_minus(coor, dis):
    '''从路径终点加上一段距离'''
    x1, y1, x2, y2 = coor[0][0], coor[0][1], coor[1][0], coor[1][1]
    if x2 - x1 == 0:
        alf1 = pi / 2.0 if y1 - y2 < 0 else -pi / 2.0
    elif y2 - y1 == 0:
        alf1 = 0 if x1 - x2 < 0 else pi
    else:
        alf1 = atan2((y2 - y1), (x2 - x1))
    alf1 = 2 * pi + alf1 if alf1 < 0 else alf1

    k = alf1  # 斜率夹角
    X = x2 - dis * cos(k)
    Y = y2 - dis * sin(k)
    return [(X, Y), (x2, y2)]


def repeat_delete(LPstpara):
    '''对标准化后数据集中的连续直线进行合并处理'''
    i = 0  # 如果真的是绝对直线连接，合并一下
    while i < len(LPstpara):
        if LPstpara[i][1] == 1:
            j = i
            while j + 1 < len(LPstpara) and LPstpara[j + 1][1] == 1:
                del LPstpara[i]
        i += 1
    return LPstpara


# 获取初始化信号控制的地图信息
def traffic_control_map():
    LSIDlist = trafficlight.getIDList()  # 信号灯IDlist
    LListB = map_topo._LListB[map_topo._VClass[0]]
    for LSID in LSIDlist:
        con_link_list = trafficlight.getControlledLinks(LSID)  # 当前信号ID控制的Links列表
        for i in range(len(con_link_list)):  # 这里用con_link的原因是，con_lane的数量和state的数量对不上
            for j in con_link_list[i]:
                now_laneID = j[0]
                if now_laneID in LListB:  # 如果是机动车的基本类型车道
                    roadID = laneIDstr_resolve(now_laneID, 'roadID')  # 获取路段ID
                    if roadID not in map_topo._RS:  # 如果不在列表中，添加上，用于路口处禁止换道的判断
                        map_topo._RS.append(roadID)

                    target_roadID = laneIDstr_resolve(j[1], 'roadID')  # 所控制link的目标roadID
                    if now_laneID not in map_topo._LS.keys():  # 如果不在列表中，添加上，用于路口闯红灯检测
                        map_topo._LS[now_laneID] = [[LSID, i, target_roadID]]  # 新建一个字典
                    else:  # 如果在列表中，说明受到了多个信号，或者一个信号多个相位的控制
                        map_topo._LS[now_laneID].append([LSID, i, target_roadID])  # 增加记录


if __name__ == "__main__":
    pass