import omni.isaac.lab.terrains as terrain

TERRAIN1_CFG = terrain.TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=10,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    difficulty_range=(0.0, 1.0),
    use_cache=False,
    sub_terrains={  # set the proportions based on the number of terrains given
        "flat": terrain.MeshPlaneTerrainCfg(proportion=0.33),
        "random_rough": terrain.HfRandomUniformTerrainCfg(
            proportion=0.33, noise_range=(0.02, 0.05), noise_step=0.02, border_width=0.25
        ),
        "slope": terrain.HfPyramidSlopedTerrainCfg(proportion=0.33, slope_range=(0.1, 0.3), platform_width=0.4),
    },
    curriculum=True,
)

TERRAIN1_PLAY_CFG = terrain.TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=4,
    num_cols=4,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    difficulty_range=(0.0, 1.0),
    use_cache=False,
    sub_terrains={ # set the proportions based on the number of terrains given
        "flat": terrain.MeshPlaneTerrainCfg(proportion=0.33),
        "random_rough": terrain.HfRandomUniformTerrainCfg(
            proportion=0.33, noise_range=(0.02, 0.05), noise_step=0.02, border_width=0.25
        ),
        "slope": terrain.HfPyramidSlopedTerrainCfg(proportion=0.33, slope_range=(0.1, 0.3), platform_width=0.4),
    },
    curriculum=False,
)
