import numpy as np
import torch
import torch.nn.functional as F

from pursuer.planner.dataLoader import get_encoder_input, receptive_field

device = 'cuda' if torch.cuda.is_available() else 'cpu'


def getHashTable(mapSize):
    """
    Return the hashTable for the given maps
    NOTE: This hastable only works for the  patch_embedding network defined in the
    transformers/Models.py file.
    :param mapSize: The size of the maps
    :returns list: the hashTable to convert 1D token index to 2D image positions
    """
    H, W = mapSize
    Hhat = np.floor((H - 8) / 4) - 1
    What = np.floor((W - 8) / 4) - 1
    tokenH = int((Hhat + 6) // 5)
    tokenW = int((What + 6) // 5)
    return [(20 * r + 4, 20 * c + 4) for c in range(tokenH) for r in range(tokenW)]


def get_patch(model, start_pos, goal_pos, input_map):
    """
    Return the patch maps for the given start and goal position, and the network
    architecture.
    :param model:
    :param start:
    :param goal:
    :param input_map:
    """
    # Identitfy Anchor points
    encoder_input = get_encoder_input(input_map, goal_pos, start_pos)
    hashTable = getHashTable(input_map.shape)
    predVal = model(encoder_input[None, :].float().to(device))
    predClass = predVal[0, :, :].max(1)[1]

    predProb = F.softmax(predVal[0, :, :], dim=1)
    possAnchor = [hashTable[i] for i, label in enumerate(predClass) if label == 1]

    # Generate Patch Maps
    patch_map = np.zeros_like(input_map)
    map_size = input_map.shape
    for pos in possAnchor:
        goal_start_x = max(0, pos[0] - receptive_field // 2)
        goal_start_y = max(0, pos[1] - receptive_field // 2)
        goal_end_x = min(map_size[1], pos[0] + receptive_field // 2)
        goal_end_y = min(map_size[0], pos[1] + receptive_field // 2)
        patch_map[goal_start_y:goal_end_y, goal_start_x:goal_end_x] = 1.0
    return patch_map, predProb
