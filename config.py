import numpy as np


def get_config_by_name(config_name):
    configs = None
    if config_name == 'debug':
        configs = {
            'dt': 0.02,
            'strategy': 'naive',
            'corners': [
                np.array([-8.64, -8.64]),
                np.array([-8.64, 8.64]),
                np.array([8.64, -8.64]),
                np.array([8.64, 8.64]),
            ],
            'velocity_component': {
                'avoid_hitting_police': 1,
                'avoid_hitting_captured_baddies': 1,
                'avoid_hitting_obstacles': 1,
                'chase_baddy': 1,
                'avoid_hitting_walls': 1,
                'avoid_hitting_baddies': 1,
                'escape_from_police': 1,
                'avoid_corners': 1,
            },
            'robots': [
                {
                    'name': 'robot1',
                    'type': 'police',
                    'epsilon': 0.6,
                    'max_speed': 0.4,
                    'visibility': -1,
                },
                {
                    'name': 'robot2',
                    'type': 'police',
                    'epsilon': 0.6,
                    'max_speed': 0.4,
                    'visibility': -1,
                },
                {
                    'name': 'robot3',
                    'type': 'baddy',
                    'epsilon': 0.6,
                    'max_speed': 0.9,
                    'visibility': -1,
                },
                {
                    'name': 'robot4',
                    'type': 'baddy',
                    'epsilon': 0.6,
                    'max_speed': 0.9,
                    'visibility': -1,
                },
            ],
            'obstacles': [
                {
                    'name': 'cylinder1',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([3, 2], dtype=np.float32),
                        'radius': 1.5,
                    }
                },
                {
                    'name': 'cylinder2',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-1, -5.5], dtype=np.float32),
                        'radius': 1.3,
                    }
                },
                {
                    'name': 'cylinder3',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-4, 3.5], dtype=np.float32),
                        'radius': 1.2,
                    }
                },
                {
                    'name': 'walls',
                    'params': {
                        'type': 'square_wall',
                        'position': np.array([-8.64, -8.64], dtype=np.float32),
                        'dx': 8.64 * 2,
                        'dy': 8.64 * 2,
                    }
                }
            ]
        }
    elif config_name == '2police2baddy':
        configs = {
            'dt': 0.01,
            'strategy': 'naive',
            'corners': [
                np.array([-8.64, -8.64]),
                np.array([-8.64, 8.64]),
                np.array([8.64, -8.64]),
                np.array([8.64, 8.64]),
            ],
            'velocity_component': {
                'avoid_hitting_police': 1,
                'avoid_hitting_captured_baddies': 1,
                'chase_baddy': 1,
                'avoid_hitting_walls': 1,
                'avoid_hitting_obstacles': 1,
                'avoid_hitting_baddies': 1,
                'escape_from_police': 1,
                'avoid_corners': 1,
            },
            'robots': [
                {
                    'name': 'robot1',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.2,
                    'visibility': -1,
                },
                {
                    'name': 'robot2',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.2,
                    'visibility': -1,
                },
                {
                    'name': 'robot3',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
                {
                    'name': 'robot4',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
            ],
            'obstacles': [
                {
                    'name': 'cylinder1',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([3, 2], dtype=np.float32),
                        'radius': 1.5,
                    }
                },
                {
                    'name': 'cylinder2',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-1, -5.5], dtype=np.float32),
                        'radius': 1.3,
                    }
                },
                {
                    'name': 'cylinder3',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-4, 3.5], dtype=np.float32),
                        'radius': 1.2,
                    }
                },
                {
                    'name': 'walls',
                    'params': {
                        'type': 'square_wall',
                        'position': np.array([-8.64, -8.64], dtype=np.float32),
                        'dx': 8.64 * 2,
                        'dy': 8.64 * 2,
                    }
                }
            ]
        }
    elif config_name == '3police3baddy':
        configs = {
            'dt': 0.01,
            'strategy': 'naive',
            'corners': [
                np.array([-8.64, -8.64]),
                np.array([-8.64, 8.64]),
                np.array([8.64, -8.64]),
                np.array([8.64, 8.64]),
            ],
            'velocity_component': {
                'avoid_hitting_police': 1,
                'avoid_hitting_captured_baddies': 1,
                'chase_baddy': 1,
                'avoid_hitting_walls': 1,
                'avoid_hitting_obstacles': 1,
                'avoid_hitting_baddies': 1,
                'escape_from_police': 1,
                'avoid_corners': 1,
            },
            'robots': [
                {
                    'name': 'robot1',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.2,
                    'visibility': -1,
                },
                {
                    'name': 'robot2',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.2,
                    'visibility': -1,
                },
                {
                    'name': 'robot3',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
                {
                    'name': 'robot4',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
                {
                    'name': 'robot5',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
                {
                    'name': 'robot6',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
            ],
            'obstacles': [
                {
                    'name': 'cylinder1',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([3, 2], dtype=np.float32),
                        'radius': 1.5,
                    }
                },
                {
                    'name': 'cylinder2',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-1, -5.5], dtype=np.float32),
                        'radius': 1.3,
                    }
                },
                {
                    'name': 'cylinder3',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-4, 3.5], dtype=np.float32),
                        'radius': 1.2,
                    }
                },
                {
                    'name': 'walls',
                    'params': {
                        'type': 'square_wall',
                        'position': np.array([-8.64, -8.64], dtype=np.float32),
                        'dx': 8.64 * 2,
                        'dy': 8.64 * 2,
                    }
                }
            ]
        }
    elif config_name == '2police3baddy':
        configs = {
            'dt': 0.01,
            'strategy': 'naive',
            'corners': [
                np.array([-8.64, -8.64]),
                np.array([-8.64, 8.64]),
                np.array([8.64, -8.64]),
                np.array([8.64, 8.64]),
            ],
            'velocity_component': {
                'avoid_hitting_police': 1,
                'avoid_hitting_captured_baddies': 1,
                'chase_baddy': 1,
                'avoid_hitting_walls': 1,
                'avoid_hitting_obstacles': 1,
                'avoid_hitting_baddies': 1,
                'escape_from_police': 1,
                'avoid_corners': 1,
            },
            'robots': [
                {
                    'name': 'robot1',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.2,
                    'visibility': -1,
                },
                {
                    'name': 'robot2',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.2,
                    'visibility': -1,
                },
                {
                    'name': 'robot3',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
                {
                    'name': 'robot4',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
                {
                    'name': 'robot5',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
            ],
            'obstacles': [
                {
                    'name': 'cylinder1',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([3, 2], dtype=np.float32),
                        'radius': 1.5,
                    }
                },
                {
                    'name': 'cylinder2',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-1, -5.5], dtype=np.float32),
                        'radius': 1.3,
                    }
                },
                {
                    'name': 'cylinder3',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-4, 3.5], dtype=np.float32),
                        'radius': 1.2,
                    }
                },
                {
                    'name': 'walls',
                    'params': {
                        'type': 'square_wall',
                        'position': np.array([-8.64, -8.64], dtype=np.float32),
                        'dx': 8.64 * 2,
                        'dy': 8.64 * 2,
                    }
                }
            ]
        }
    elif config_name == '3police2baddy':
        configs = {
            'dt': 0.01,
            'strategy': 'naive',
            'corners': [
                np.array([-8.64, -8.64]),
                np.array([-8.64, 8.64]),
                np.array([8.64, -8.64]),
                np.array([8.64, 8.64]),
            ],
            'velocity_component': {
                'avoid_hitting_police': 1,
                'avoid_hitting_captured_baddies': 1,
                'chase_baddy': 1,
                'avoid_hitting_walls': 1,
                'avoid_hitting_obstacles': 1,
                'avoid_hitting_baddies': 1,
                'escape_from_police': 1,
                'avoid_corners': 1,
            },
            'robots': [
                {
                    'name': 'robot1',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.2,
                    'visibility': -1,
                },
                {
                    'name': 'robot2',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.2,
                    'visibility': -1,
                },
                {
                    'name': 'robot3',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
                {
                    'name': 'robot4',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
                {
                    'name': 'robot5',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                },
            ],
            'obstacles': [
                {
                    'name': 'cylinder1',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([3, 2], dtype=np.float32),
                        'radius': 1.5,
                    }
                },
                {
                    'name': 'cylinder2',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-1, -5.5], dtype=np.float32),
                        'radius': 1.3,
                    }
                },
                {
                    'name': 'cylinder3',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-4, 3.5], dtype=np.float32),
                        'radius': 1.2,
                    }
                },
                {
                    'name': 'walls',
                    'params': {
                        'type': 'square_wall',
                        'position': np.array([-8.64, -8.64], dtype=np.float32),
                        'dx': 8.64 * 2,
                        'dy': 8.64 * 2,
                    }
                }
            ]
        }
    elif config_name == '2police1baddy':
        configs = {
            'dt': 0.01,
            'strategy': 'naive',
            'corners': [
                np.array([-8.64, -8.64]),
                np.array([-8.64, 8.64]),
                np.array([8.64, -8.64]),
                np.array([8.64, 8.64]),
            ],
            'velocity_component': {
                'avoid_hitting_police': 1,
                'avoid_hitting_captured_baddies': 1,
                'chase_baddy': 1,
                'avoid_hitting_walls': 1,
                'avoid_hitting_obstacles': 1,
                'avoid_hitting_baddies': 1,
                'escape_from_police': 1,
                'avoid_corners': 1,
            },
            'robots': [
                {
                    'name': 'robot1',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.2,
                    'visibility': -1,
                },
                {
                    'name': 'robot2',
                    'type': 'police',
                    'epsilon': 0.2,
                    'max_speed': 0.2,
                    'visibility': -1,
                },
                {
                    'name': 'robot3',
                    'type': 'baddy',
                    'epsilon': 0.2,
                    'max_speed': 0.1,
                    'visibility': -1,
                }
            ],
            'obstacles': [
                {
                    'name': 'cylinder1',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([3, 2], dtype=np.float32),
                        'radius': 1.5,
                    }
                },
                {
                    'name': 'cylinder2',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-1, -5.5], dtype=np.float32),
                        'radius': 1.3,
                    }
                },
                {
                    'name': 'cylinder3',
                    'params': {
                        'type': 'cylinder',
                        'position': np.array([-4, 3.5], dtype=np.float32),
                        'radius': 1.2,
                    }
                },
                {
                    'name': 'walls',
                    'params': {
                        'type': 'square_wall',
                        'position': np.array([-8.64, -8.64], dtype=np.float32),
                        'dx': 8.64 * 2,
                        'dy': 8.64 * 2,
                    }
                }
            ]
        }
    assert configs is not None, "can't find the requested config!"

    return configs