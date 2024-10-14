import omni.isaac.lab.terrains as terrain

TERRAIN1_CFG = terrain.TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=10,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    difficulty_range=(0.3, 1.2),
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
    sub_terrains={  # set the proportions based on the number of terrains given
        "flat": terrain.MeshPlaneTerrainCfg(proportion=0.33),
        "random_rough": terrain.HfRandomUniformTerrainCfg(
            proportion=0.33, noise_range=(0.02, 0.05), noise_step=0.02, border_width=0.25
        ),
        "slope": terrain.HfPyramidSlopedTerrainCfg(proportion=0.33, slope_range=(0.1, 0.3), platform_width=0.4),
    },
    curriculum=False,
)

TERRAIN2_CFG = terrain.TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=10,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    difficulty_range=(0.3, 1.2),
    use_cache=False,
    sub_terrains={  # set the proportions based on the number of terrains given
        "flat": terrain.MeshPlaneTerrainCfg(proportion=0.05),
<<<<<<< HEAD
        # "steps": terrain.HfInvertedPyramidStairsTerrainCfg(proportion=0.45,
        #                                                    step_height_range=(0.1, 0.2),step_width=0.2, border_width=0.25),
=======
        "steps": terrain.HfInvertedPyramidStairsTerrainCfg(
            proportion=0.45, step_height_range=(0.1, 0.2), step_width=0.2, border_width=0.25
        ),
>>>>>>> 9c329ad3971243fe25cccd34ca0dbba8766abfe4
        "random_rough": terrain.HfRandomUniformTerrainCfg(
            proportion=0.25, noise_range=(0.02, 0.05), noise_step=0.02, border_width=0.25
        ),
        "slope": terrain.HfPyramidSlopedTerrainCfg(
            proportion=0.25, slope_range=(0.1, 0.3), platform_width=0.4, border_width=0.25
        ),
        "stones": terrain.HfSteppingStonesTerrainCfg(
<<<<<<< HEAD
            proportion=0.10,
            stone_height_max=0.05,
            stone_width_range=(0.35, 0.4),
            stone_distance_range=(0.075, 0.075),
=======
            proportion=0.15,
            stone_height_max=0.15,
            stone_width_range=(0.15, 0.20),
            stone_distance_range=(0.09, 0.095),
>>>>>>> 9c329ad3971243fe25cccd34ca0dbba8766abfe4
        ),
    },
    curriculum=True,
)

TERRAIN2_PLAY_CFG = terrain.TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=4,
    num_cols=4,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    difficulty_range=(0.0, 1.0),
    use_cache=False,
    sub_terrains={  # set the proportions based on the number of terrains given
        "flat": terrain.MeshPlaneTerrainCfg(proportion=0.05),
<<<<<<< HEAD
        # "steps": terrain.HfInvertedPyramidStairsTerrainCfg(proportion=0.25,
        #                                                    step_height_range=(0.1, 0.2),step_width=0.2),
=======
        "steps": terrain.HfInvertedPyramidStairsTerrainCfg(
            proportion=0.25, step_height_range=(0.1, 0.2), step_width=0.2
        ),
>>>>>>> 9c329ad3971243fe25cccd34ca0dbba8766abfe4
        "random_rough": terrain.HfRandomUniformTerrainCfg(
            proportion=0.25, noise_range=(0.02, 0.05), noise_step=0.02, border_width=0.25
        ),
        "slope": terrain.HfPyramidSlopedTerrainCfg(proportion=0.25, slope_range=(0.1, 0.3), platform_width=0.4),
        "stones": terrain.HfSteppingStonesTerrainCfg(
<<<<<<< HEAD
            proportion=0.05,
            stone_height_max=0.05,
            stone_width_range=(0.45, 0.5),
            stone_distance_range=(0.10, 0.10),
=======
            proportion=0.15,
            stone_height_max=0.15,
            stone_width_range=(0.15, 0.20),
            stone_distance_range=(0.0, 0.095),
>>>>>>> 9c329ad3971243fe25cccd34ca0dbba8766abfe4
        ),
    },
    curriculum=False,
)
