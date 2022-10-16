import collections
import numpy as np
import os
import pandas as pd
from models.model_loading import MODEL_LIST, load_pvr_transforms

CAMERA_IDS = {
    "818312070212": 0,
}

def validate_images(path, csv_data):
    color_cam_fn = {i:[] for i in CAMERA_IDS.values()}
    depth_cam_fn = {i:[] for i in CAMERA_IDS.values()}

    for dp_idx, timestamps in enumerate(csv_data['cam']):
        if pd.isna(timestamps) or len(timestamps.split('-')) != 2:
            for i in CAMERA_IDS.values():
                color_cam_fn[i].append(np.nan)
                depth_cam_fn[i].append(np.nan)
            continue

        datapoint = timestamps.split('-')
        for cam_id, i in CAMERA_IDS.items():
            cimage_fn = f"c{i}-{cam_id}-{datapoint[i*2]}-color.jpeg"
            color_cam_fn[i].append(cimage_fn
                if os.path.isfile(os.path.join(path, cimage_fn))
                else np.nan
            )
            dimage_fn = f"c{i}-{cam_id}-{datapoint[i*2+1]}-depth.jpeg"
            depth_cam_fn[i].append(dimage_fn
                if os.path.isfile(os.path.join(path, dimage_fn))
                else np.nan
            )

    for i in CAMERA_IDS.values():
        csv_data[f"cam{i}c"] = color_cam_fn[i]
        csv_data[f"cam{i}d"] = depth_cam_fn[i]

    old_data = csv_data['cam'].copy()
    # Remove datapoints missing any image
    csv_data.dropna(inplace=True)
    valid_data = csv_data[csv_data['robocmd'] != 'None']

    if len(valid_data) < 10: # Constraint on trajectory length
        print('Too short after image validation',path)
        return None
    return valid_data


def get_reward(path):
    try:
        csv_log = os.path.join(path, 'log.csv')
        csv_data = pd.read_csv(csv_log, names=['timestamp','robostate','robocmd', 'reward', 'cam'])

        reward = csv_data.iloc[-1].reward
        if reward == "None" or int(reward) == -1:
            print('Bad reward',path)
            return None, None
    except Exception as e:
        # File log.csv did not exist or could not be read
        print('Issue reading log',path)
        return None, None

    valid_data = validate_images(path, csv_data)
    if valid_data is None:
        return None, None
    return int(reward), valid_data


class Namespace(collections.MutableMapping):
    """Utility class to convert a (nested) dictionary into a (nested) namespace.
    >>> x = Namespace({'foo': 1, 'bar': 2})
    >>> x.foo
    1
    >>> x.bar
    2
    >>> x.baz
    Traceback (most recent call last):
        ...
    KeyError: 'baz'
    >>> x
    {'foo': 1, 'bar': 2}
    >>> (lambda **kwargs: print(kwargs))(**x)
    {'foo': 1, 'bar': 2}
    >>> x = Namespace({'foo': {'a': 1, 'b': 2}, 'bar': 3})
    >>> x.foo.a
    1
    >>> x.foo.b
    2
    >>> x.bar
    3
    >>> (lambda **kwargs: print(kwargs))(**x)
    {'foo': {'a': 1, 'b': 2}, 'bar': 3}
    >>> (lambda **kwargs: print(kwargs))(**x.foo)
    {'a': 1, 'b': 2}
    """

    def __init__(self, data):
        self._data = data

    def __getitem__(self, k):
        return self._data[k]

    def __setitem__(self, k, v):
        self._data[k] = v

    def __delitem__(self, k):
        del self._data[k]

    def __iter__(self):
        return iter(self._data)

    def __len__(self): 
        return len(self._data)

    def __getattr__(self, k):
        if not k.startswith('_'):
            if k not in self._data:
                return Namespace({})
            v = self._data[k]
            if isinstance(v, dict):
                v = Namespace(v)
            return v

        if k not in self.__dict__:
            raise AttributeError("'Namespace' object has no attribute '{}'".format(k))

        return self.__dict__[k]

    def __repr__(self):
        return repr(self._data)


def generate_camera_transforms(config):
    from torchvision import transforms
    if 'rb2' in config.agent.custom_resnet_path:
        print("Using RB2 transforms - includes crop, grayscale, color jitter")
        return transforms.Compose([transforms.RandomResizedCrop((config.data.images.im_h, config.data.images.im_w), (0.8, 1.0)),
                                   transforms.RandomGrayscale(p=0.05),
                                   transforms.ColorJitter(brightness=0.4, contrast=0.3, saturation=0.3, hue=0.3),
                                   transforms.ToTensor(),
                                   transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                                        std=[0.229, 0.224, 0.225], inplace=True)])
    # if 'r3m' in config.agent.custom_resnet_path:
    #     return transforms.Compose([
    #         transforms.Resize(256),
    #         transforms.CenterCrop(224),
    #         transforms.ToTensor(),  # this divides by 255                                                                         
    #         transforms.Normalize(mean=[0.0, 0.0, 0.0], std=[1.0/255, 1.0/255, 1.0/255]), # this will scale bact to [0-255]        
    #     ])
    # if 'moco' in config.agent.custom_resnet_path:
    #     print("Using moco transforms")
    #     return transforms.Compose([
    #         transforms.Resize(256),
    #         transforms.CenterCrop(224),
    #         transforms.ToTensor(),
    #         transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    #     ])
    if config.agent.type == 'bcimage' and config.agent.custom_resnet_path in MODEL_LIST:
        embedding_dim, transforms = load_pvr_transforms(config.agent.custom_resnet_path)
        return transforms
    if config.agent.type == 'bcimagebyol':
        print("Using BYOL transforms")
        from agents.knn_byol import byol_transforms
        return byol_transforms
    return transforms.Compose([
        transforms.Resize((config.data.images.im_h, config.data.images.im_w)),
        transforms.ToTensor(),
        # transforms.Normalize([0,0,0], [255,255,255], inplace=True),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225], inplace=True),
        ])
