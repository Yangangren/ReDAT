import logging
import os
import sys
proj_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.dirname(__file__))
sys.path.append(proj_root)

import numpy as np
from env_build.dynamics_and_models import EnvironmentModel

from algorithm.preprocessor import Preprocessor
from algorithm.utils.misc import TimerStat, args2envkwargs, judge_is_nan
import tensorflow_probability as tfp
tfd = tfp.distributions
tfb = tfp.bijectors

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class AMPCLearnerWithAttention(object):
    import tensorflow as tf
    tf.config.experimental.set_visible_devices([], 'GPU')
    tf.config.threading.set_inter_op_parallelism_threads(1)
    tf.config.threading.set_intra_op_parallelism_threads(1)

    def __init__(self, policy_cls, args):
        self.args = args
        self.policy_with_value = policy_cls(self.args)
        self.batch_data = {}
        self.all_data = {}
        self.num_rollout_list_for_policy_update = self.args.num_rollout_list_for_policy_update

        self.model = EnvironmentModel()
        self.preprocessor = Preprocessor(self.args.obs_scale, self.args.reward_scale, self.args.reward_shift,
                                         args=self.args, gamma=self.args.gamma)
        self.grad_timer = TimerStat()
        self.stats = {}
        self.info_for_buffer = {}
        self.batch_size = self.args.replay_batch_size
        self.adv_act_scale = (np.array(self.args.adv_act_bound['high'], dtype=np.float32) - np.array(self.args.adv_act_bound['low'], dtype=np.float32)) / 2.
        self.adv_act_bias = (np.array(self.args.adv_act_bound['high'], dtype=np.float32) + np.array(self.args.adv_act_bound['low'], dtype=np.float32)) / 2.

    def get_stats(self):
        return self.stats
    
    def get_states(self, mb_obs, mb_mask):
        mb_obs_others, mb_attn_weights = self.policy_with_value.compute_attn(mb_obs[:, self.args.other_start_dim:], mb_mask)
        mb_state = self.tf.concat((mb_obs[:, :self.args.other_start_dim], mb_obs_others),
                                  axis=1)
        return mb_state

    def get_info_for_buffer(self):
        return self.info_for_buffer

    def get_batch_data(self, batch_data):
        self.batch_data = {'batch_obs': batch_data[0].astype(np.float32),
                           'batch_future_n_point': batch_data[5].astype(np.float32),
                           'batch_mask': batch_data[6].astype(np.float32),
                           'batch_future_n_edge': batch_data[7].astype(np.float32),
                           }

    def get_weights(self):
        return self.policy_with_value.get_weights()

    def set_weights(self, weights):
        return self.policy_with_value.set_weights(weights)

    def punish_factor_schedule(self, ite):
        init_pf = self.args.init_punish_factor
        interval = self.args.pf_enlarge_interval
        amplifier = self.args.pf_amplifier
        pf = init_pf * self.tf.pow(amplifier, self.tf.cast(ite//interval, self.tf.float32))
        return pf

    def model_rollout_for_update(self, mb_obs, ite, mb_future_n_point, mb_mask, mb_future_n_edge):
        self.model.reset(mb_obs, mb_future_n_point, mb_future_n_edge)
        rewards_sum = self.tf.zeros((self.batch_size,))
        punish_terms_for_training_sum = self.tf.zeros((self.batch_size,))
        real_punish_terms_sum = self.tf.zeros((self.batch_size,))
        veh2veh4real_sum = self.tf.zeros((self.batch_size,))
        veh2road4real_sum = self.tf.zeros((self.batch_size,))
        veh2bike4real_sum = self.tf.zeros((self.batch_size,))
        veh2person4real_sum = self.tf.zeros((self.batch_size,))
        veh2speed4real_sum = self.tf.zeros((self.batch_size,))
        entropy_sum = self.tf.zeros((self.batch_size,))

        pf = self.punish_factor_schedule(ite)
        processed_mb_obs = self.preprocessor.tf_process_obses(mb_obs)
        mb_state = self.get_states(processed_mb_obs, mb_mask)
        obj_v_pred = self.policy_with_value.compute_obj_v(self.tf.stop_gradient(mb_state))

        for i in range(self.num_rollout_list_for_policy_update[0]):
            processed_mb_obs = self.preprocessor.tf_process_obses(mb_obs)
            mb_state = self.get_states(processed_mb_obs, mb_mask)
            actions, logps = self.policy_with_value.compute_action(mb_state)
            if self.args.noise_mode == 'adv_noise':
                adv_actions, _ = self.policy_with_value.compute_adv_action(mb_state) # todo: stop gradient?
                adv_actions = self.adv_act_scale * adv_actions + self.adv_act_bias
            elif self.args.noise_mode == 'no_noise':
                adv_actions = self.tf.zeros(shape=(self.batch_size, self.args.adv_act_dim * self.args.other_number))
            else:
                mean = self.tf.zeros(shape=(self.batch_size, self.args.adv_act_dim * self.args.other_number))
                std = self.tf.ones(shape=(self.batch_size, self.args.adv_act_dim * self.args.other_number))
                act_dist = tfp.distributions.MultivariateNormalDiag(mean, std)
                act_dist = tfp.distributions.TransformedDistribution(distribution=act_dist, bijector=tfb.Tanh())
                adv_actions = act_dist.sample()
                adv_actions = self.adv_act_scale * adv_actions + self.adv_act_bias
            adv_actions = self.args.adv_gamma ** i * adv_actions
            mb_obs, rewards, punish_terms_for_training, real_punish_term, \
                veh2veh4real, veh2road4real, veh2bike4real, veh2person4real, veh2speed4real = \
                self.model.rollout_out(actions, adv_actions)  # mb_future_n_point [#batch, 4, T]
            rewards_sum += self.preprocessor.tf_process_rewards(rewards)
            punish_terms_for_training_sum += self.args.reward_scale * punish_terms_for_training
            real_punish_terms_sum += self.args.reward_scale * real_punish_term
            veh2veh4real_sum += self.args.reward_scale * veh2veh4real
            veh2road4real_sum += self.args.reward_scale * veh2road4real
            veh2bike4real_sum += self.args.reward_scale * veh2bike4real
            veh2person4real_sum += self.args.reward_scale * veh2person4real
            veh2speed4real_sum += self.args.reward_scale * veh2speed4real
            entropy_sum += -logps
        # obj v loss
        obj_v_loss = self.tf.reduce_mean(self.tf.square(obj_v_pred - self.tf.stop_gradient(-rewards_sum)))
        # con_v_loss = self.tf.reduce_mean(self.tf.square(con_v_pred - self.tf.stop_gradient(real_punish_terms_sum)))

        # pg loss
        obj_loss = -self.tf.reduce_mean(rewards_sum)
        punish_term_for_training = self.tf.reduce_mean(punish_terms_for_training_sum)
        punish_loss = self.tf.stop_gradient(pf) * punish_term_for_training
        pg_loss = obj_loss + punish_loss

        real_punish_term = self.tf.reduce_mean(real_punish_terms_sum)
        veh2veh4real = self.tf.reduce_mean(veh2veh4real_sum)
        veh2road4real = self.tf.reduce_mean(veh2road4real_sum)
        veh2bike4real = self.tf.reduce_mean(veh2bike4real_sum)
        veh2person4real = self.tf.reduce_mean(veh2person4real_sum)
        veh2speed4real = self.tf.reduce_mean(veh2speed4real_sum)

        policy_entropy = self.tf.reduce_mean(entropy_sum) / self.num_rollout_list_for_policy_update[0]

        return obj_v_loss, obj_loss, punish_term_for_training, punish_loss, pg_loss,\
               real_punish_term, veh2veh4real, veh2road4real, veh2bike4real, veh2person4real, veh2speed4real, pf, policy_entropy

    # @tf.function
    def forward_and_backward(self, mb_obs, ite, mb_future_n_point, mb_mask, mb_future_n_edge):
        with self.tf.GradientTape(persistent=True) as tape:
            obj_v_loss, obj_loss, punish_term_for_training, punish_loss, pg_loss, \
            real_punish_term, veh2veh4real, veh2road4real, veh2bike4real, veh2person4real, veh2speed4real, pf, policy_entropy\
                = self.model_rollout_for_update(mb_obs, ite, mb_future_n_point, mb_mask, mb_future_n_edge)
            adv_pg_loss = -punish_loss

        if self.args.noise_mode == 'adv_noise':
            with self.tf.name_scope('policy_gradient') as scope:
                pg_grad = tape.gradient(pg_loss, self.policy_with_value.policy.trainable_weights)
                attn_net_grad = tape.gradient(pg_loss, self.policy_with_value.attn_net.trainable_weights)
                adv_pg_grad = tape.gradient(adv_pg_loss, self.policy_with_value.adv_policy.trainable_weights)
        else:
            with self.tf.name_scope('policy_gradient') as scope:
                pg_grad = tape.gradient(pg_loss, self.policy_with_value.policy.trainable_weights)
                attn_net_grad = tape.gradient(pg_loss, self.policy_with_value.attn_net.trainable_weights)
                adv_pg_grad = [self.tf.zeros_like(pg_grad[0])]
        with self.tf.name_scope('obj_v_gradient') as scope:
            obj_v_grad = tape.gradient(obj_v_loss, self.policy_with_value.obj_v.trainable_weights)

        return pg_grad, adv_pg_grad, obj_v_grad, attn_net_grad, obj_v_loss, obj_loss, \
               punish_term_for_training, punish_loss, pg_loss,\
               real_punish_term, veh2veh4real, veh2road4real, veh2bike4real, veh2person4real, veh2speed4real, pf, policy_entropy

    def compute_gradient(self, samples, iteration):
        self.get_batch_data(samples)
        mb_obs = self.tf.constant(self.batch_data['batch_obs'])
        mb_future_n_point = self.tf.constant(self.batch_data['batch_future_n_point'])
        iteration = self.tf.convert_to_tensor(iteration, self.tf.int32)
        mb_mask = self.tf.constant(self.batch_data['batch_mask'])
        mb_future_n_edge = self.tf.constant(self.batch_data['batch_future_n_edge'])

        with self.grad_timer:
            pg_grad, adv_pg_grad, obj_v_grad, attn_net_grad, obj_v_loss, obj_loss, \
            punish_term_for_training, punish_loss, pg_loss, \
            real_punish_term, veh2veh4real, veh2road4real, veh2bike4real, veh2person4real, veh2speed4real, pf, policy_entropy =\
                self.forward_and_backward(mb_obs, iteration, mb_future_n_point, mb_mask, mb_future_n_edge)

            pg_grad, pg_grad_norm = self.tf.clip_by_global_norm(pg_grad, self.args.gradient_clip_norm)
            adv_pg_grad, adv_pg_grad_norm = self.tf.clip_by_global_norm(adv_pg_grad, self.args.gradient_clip_norm)
            obj_v_grad, obj_v_grad_norm = self.tf.clip_by_global_norm(obj_v_grad, self.args.gradient_clip_norm)
            attn_net_grad, attn_net_grad_norm = self.tf.clip_by_global_norm(attn_net_grad, self.args.gradient_clip_norm)
            # con_v_grad, con_v_grad_norm = self.tf.clip_by_global_norm(con_v_grad, self.args.gradient_clip_norm)

        self.stats.update(dict(
            iteration=iteration,
            grad_time=self.grad_timer.mean,
            obj_loss=obj_loss.numpy(),
            punish_term_for_training=punish_term_for_training.numpy(),
            real_punish_term=real_punish_term.numpy(),
            veh2veh4real=veh2veh4real.numpy(),
            veh2road4real=veh2road4real.numpy(),
            veh2bike4real=veh2bike4real.numpy(),
            veh2person4real=veh2person4real.numpy(),
            veh2speed4real=veh2speed4real.numpy(),
            punish_loss=punish_loss.numpy(),
            pg_loss=pg_loss.numpy(),
            obj_v_loss=obj_v_loss.numpy(),
            # con_v_loss=con_v_loss.numpy(),
            punish_factor=pf.numpy(),
            pg_grads_norm=pg_grad_norm.numpy(),
            adv_pg_grads_norm=adv_pg_grad_norm.numpy(),
            obj_v_grad_norm=obj_v_grad_norm.numpy(),
            attn_net_grad_norm=attn_net_grad_norm.numpy(),
            policy_entropy=policy_entropy.numpy(),
            # con_v_grad_norm=con_v_grad_norm.numpy()
        ))

        grads = obj_v_grad + pg_grad + attn_net_grad + adv_pg_grad
        return list(map(lambda x: x.numpy(), grads))


if __name__ == '__main__':
    pass
