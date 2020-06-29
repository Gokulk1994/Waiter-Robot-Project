Include = '../../robotics-course/rai-robotModels/pr2/pr2_clean.g'
Include = 'pr2_modifications.g'
Include = 'panda_arms.g'
Include = 'camera.g'


shelf(table) {  mesh:'Table.dae', X:<t(1.3 2.5 -0.05) d(90 1 0 0)>, contact, meshscale: .52, color:[0,0,1]}

tray (base_footprint){shape:box, size:[0.5, 0.7,0.05],, mass:1.0 X:<[0.63, 0.0, 0.65, 1,0,0,0]>, color:[0,0,0],friction:10.9,contact}

Edit worldTranslationRotation { Q:<t(2.0 1.3 0.05) d(90 0 0 1)> gains=[1 1] ctrl_limits=[1 1 1] ctrl_H=10 base }


coffe_table {
    X:<t(-1 0.85 -0.061) d(90 1 0 0)>,
    color:[0.6 0 0],
    mesh:'Table.dae',  #mesh file
    #contact:1,
    meshscale: .52 #make it a bit smaller
}

coffe_table_1 {
    X:<t(-1 -0.85 -0.061) d(90 1 0 0) >,
    color:[0 0.6 0],
    mesh:'Table.dae',  #mesh file
    #contact:1,
    meshscale: .52
}


dyna_glass {
	shape:mesh,
	X:<t(1.1 2.5 0.50) d(90 1 0 0)>,,
    color:[0. 1 0],
    mesh:'water.dae',
    #contact:1,
    meshscale: 0.06,
    friction:10.0,

}


chair_11 {
	shape:mesh,
	X:<t(-1 1.6 0) d(90 1 0 0) d(-90 0 1 0)>,,
    color:[0.6 0.6 0],
    mesh:'chair.dae',
    #contact:1,
    meshscale: 0.2,
    friction:10.0,

}

chair_21 {
	shape:mesh,
	X:<t(-1.6 -0.85 0) d(90 1 0 0)>,,
    color:[0.6 0.6 0],
    mesh:'chair.dae',
    #contact:1,
    meshscale: 0.2,
    friction:10.0,

}

chair_22 {
	shape:mesh,
	X:<t(-1 -0.3 0) d(90 1 0 0) d(-90 0 1 0)>,,
    color:[0.6 0.6 0],
    mesh:'chair.dae',
    #contact:1,
    meshscale: 0.2,
    friction:10.0,

}

human_scene(table) { mesh: 'human_scene.stl', X:<t(-2.4 0.85 0) d(-90 0 0 1) >,  meshscale: .02 }

dyna_coffe_mug {
    shape:mesh,
    X:<t(1.4 2.5 0.63) d(90 1 0 0)>,,   #absolute pose: translation by (0,0,1)
    color:[0. 1 1],
    mesh:'mug.dae',
    contact:1,
    meshscale: 2.2,
    friction:10.0
}


dyna_coffe_mug1 {
    shape:mesh,
    X:<t(1.4 2.2 0.63) d(90 1 0 0)>,,   #absolute pose: translation by (0,0,1)
    color:[0. 1 1],
    mesh:'mug.dae',
    contact:1,
    meshscale: 2.2,
    friction:10.0
}


dyna_coffe_mug2 {
    shape:mesh,
    X:<t(1.4 2.8 0.63) d(90 1 0 0)>,,   #absolute pose: translation by (0,0,1)
    color:[0. 1 1],
    mesh:'mug.dae',
    contact:1,
    meshscale: 2.2,
    friction:10.0
}

