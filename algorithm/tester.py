import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class Tester(object):
    def __init__(self, policy_cls, evaluator_cls, args):
        self.args = args
        self.evaluator = evaluator_cls(policy_cls, self.args.env_id, self.args)

    def evaluate_saved_model(self, model_load_dir, iteration):
        self.evaluator.evaluate_saved_model(model_load_dir, iteration)

    def test(self):
        logger.info('testing beginning')
        for ite in self.args.test_iter_list:
            logger.info('testing {}-th iter model'.format(ite))
            model_load_dir = self.args.test_dir + '/models'
            self.evaluate_saved_model(model_load_dir, ite)
            self.evaluator.run_evaluation(ite)

