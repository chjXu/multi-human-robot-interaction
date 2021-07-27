import argparse
import time

import torch
from cvpack.torch_modeling.engine.engine import Engine
from cvpack.utils.pyt_utils import ensure_dir

from tensorboardX import SummaryWriter

from config import cfg
from models.with_mobilenet import PoseEstimationWithMobileNet
from lib.utils.solver import make_lr_scheduler, make_optimizer



def main():
    parser = argparse.ArgumentParser()

    with Engine(cfg, custom_parser = parser) as engine:
        logger = engine.setup_log(name='train', 
                log_dir=cfg.OUTPUT_DIR, file_name='log.txt')

        args = engine.args
        ensure_dir(cfg.OUTPUT_DIR)

        model = PoseEstimationWithMobileNet(num_refinement_stages=cfg.NUM_REFINEMENT_STAGES)
        device = torch.device(cfg.MODEL.DEVICE)
        model.to(device)

        num_gpu = len(engine.devices)

        cfg.SOLVER.CHECKPOINT_PERIOD = \
                int(cfg.SOLVER.CHECKPOINT_PERIOD * 8 / num_gpu)
        cfg.SOLVER.MAX_ITER = int(cfg.SOLVER.MAX_ITER * 8 / num_gpu)

        optimizer = make_optimizer(cfg, model, num_gpu)
        scheduler = make_lr_scheduler(cfg, optimizer)



    # load_datasets
    data_loader = get_train_loader(cfg, num_gpu=num_gpu, is_dist=engine.distributed,
                                   use_argumentation=True, with_mds=cfg.WITH_MDS)


    # training
    logger.info("\n\nStart training with pytorch version {}".format(
            torch.__version__))


    max_iter = len(data_loader)

    PoseEstimationWithMobileNet.train()

    time1 = time.time()

    for iteration, (image, valids, labels) in enumerate(
                    data_loader, engine.state.iteration):
        iteration = iteration + 1
        image = image.to(device)
        valids = valids.to(device)
        labels = labels.to(device)

        loss_dict = model(image, valids, labels)
        losses = loss_dict['total_loss']

        optimizer.zero_grad()
        losses.backward()
        optimizer.step()
        scheduler.step()


if __name__=='__main__':
    main()