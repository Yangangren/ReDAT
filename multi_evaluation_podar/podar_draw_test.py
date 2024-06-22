import pickle
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from multi_evaluation_podar.podar import PODAR, podar_render

def local_render():
    with open(r'/home/zhanguojian/Toyota_Project/results/FHADP_TOYOTA/230105-191241/230208-170129IDCevaluation/batch0/episode0_vehicles.pkl', 'rb') as f:
        data = pickle.load(f)

    step = 151
    data_draw = data[step]

    def rotation(l, w, phi):
        """phi: rad"""
        diff_x = l * np.cos(phi) - w * np.sin(phi)
        diff_y = l * np.sin(phi) + w * np.cos(phi)
        return (diff_x, diff_y)

    def _draw_rotate_rec(veh, ec):
        diff_x, diff_y = rotation(-veh.length / 2, -veh.width / 2, veh.phi0)
        rec = patches.Rectangle((veh.x0 + diff_x, veh.y0 + diff_y), veh.length, veh.width, 
            angle=veh.phi0/np.pi*180, ec=ec, fc='white')
        return rec

    fig = plt.figure(figsize=(8, 8))
    ax = plt.subplot()

    x_min, x_max, y_min, y_max = -10000, 10000, -10000, 10000
    ego_name = list(data_draw.ego.keys())[0]
    for _name, _veh in {**data_draw.ego, **data_draw.obj}.items():
        rec_handle = _draw_rotate_rec(_veh, 'black' if _name != ego_name else 'red')
        ax.add_patch(rec_handle)    
        ax.text(_veh.x0, _veh.y0, 'id={}, r={:.2f}, v={:.1f}'.format(_name, 
                    (data_draw.ego[ego_name].risk2obj[_name] if _name != ego_name else np.max(data_draw.ego[ego_name].risk)),
                    _veh.speed))
        ax.scatter(_veh.x0, _veh.y0, c='black', s=5)
        ax.plot(_veh.x_pred, _veh.y_pred, linestyle='--')
        x_min, x_max, y_min, y_max = np.min([x_min, _veh.x0]), np.max([x_max, _veh.x0]), np.min([y_min, _veh.y0]), np.max([y_max, _veh.y0])
        print('id={}, x={}, y={}, v={}, phi={}, risk={:.2f}, l={}, w={}, a={:.3f}'.format(_veh.name, _veh.x0, _veh.y0, _veh.speed, _veh.phi0,
                    data_draw.ego[ego_name].risk2obj[_name] if _name != ego_name else np.max(data_draw.ego[ego_name].risk),
                    _veh.length, _veh.width, _veh.a0))

    plt.xlim(x_min - 10, x_max + 10)
    plt.ylim(y_min - 10, y_max + 10)
    plt.axis('equal')

    plt.savefig(r'/home/zhanguojian/Toyota_Project/results/FHADP_TOYOTA/230105-191241/230208-170129IDCevaluation/batch0/0-{}.jpg'.format(step))

def re_draw(path):
    episode = 8
    with open(path + r'episode{}_vehicles.pkl'.format(episode), 'rb') as f:
        data = pickle.load(f)
    # step = 138
    for step in [151]: # np.arange(90, 190, 5)
        data_draw: PODAR = data[step]
        ego_name = list(data_draw.ego.keys())[0]
        podar = PODAR()
        podar.add_ego(name=ego_name, length=5.0, width=1.8, mass=1.8, max_dece=3.5)
        podar.ego_surr_list = data_draw.ego_surr_list
        podar.update_ego(name=ego_name, x0=data_draw.ego[ego_name].x0, y0=data_draw.ego[ego_name].y0, speed=data_draw.ego[ego_name].speed, phi0=data_draw.ego[ego_name].phi0, a0=data_draw.ego[ego_name].a0)
        for k, v in data_draw.obj.items():
            podar.add_obj(name=k, x0=v.x0, y0=v.y0, speed=v.speed, phi0=v.phi0, a0=v.a0, length=v.length, width=v.width)
        risk, _, _ = podar.estimate_risk(ego_name)
        podar_render(podar, ego_name)

        plt.title('episode 0, step {}, risk={:.3f}'.format(step, risk))
        plt.savefig(path + '{}-{}.jpg'.format(episode, step))

# local_render()
re_draw(r'/home/zhanguojian/Toyota_Project/results/INFADP_TOYOTA/230210-133004/230213-171017IDCevaluation/batch0/')

#  /home/zhanguojian/Toyota_Project/results/FHADP_TOYOTA/230105-191241/230207-115857IDCevaluation/batch1/  14  15
# /home/zhanguojian/Toyota_Project/results/FHADP_TOYOTA/230105-191241/230207-115857IDCevaluation/batch3/ 31 