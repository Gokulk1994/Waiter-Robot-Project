Include = '../../robotics-course/rai-robotModels/pr2/pr2_clean.g'
Include = 'pr2_modifications.g'
Include = 'panda_arms.g'
Include = 'camera.g'


shelf(table) {  mesh:'Table.dae', X:<t(1.3 2.5 -0.05) d(90 1 0 0)>, meshscale: .52, color:[0,0,1]}

inventory(table) {  mesh:'Table.dae', X:<t(4.0 3.7 -0.05) d(90 1 0 0)>, meshscale: .52, color:[1,1,1]}

tray (base_footprint){shape:box, size:[0.5, 0.7,0.05],, mass:1.0 X:<[0.63, 0.0, 0.65, 1,0,0,0]>, color:[0,0,0],friction:10.9,contact}

Edit worldTranslationRotation { Q:<t(2.2 1.3 0.05) d(90 0 0 1)> gains=[1 1] ctrl_limits=[1 1 1] ctrl_H=10 base }

sprite_1 {  shape:cylinder, size:[0.22, 0.07],, contact, mass:100.0 X:<[4.1, 3.3, 0.6, -7.41170883e-01, -2.67996453e-04, 1.97769841e-05,  6.71316564e-01]>, color:[0,1,0], friction:10.0 }

sprite_2 {  shape:cylinder, size:[0.22, 0.07],,contact, mass:100.0 X:<[4.1, 3.6, 0.6, -7.41170883e-01, -2.67996453e-04, 1.97769841e-05,  6.71316564e-01]>, color:[0,1,0], friction:10.0 }

sprite_3 {  shape:cylinder, size:[0.22, 0.07],,contact, mass:100.0 X:<[4.1, 3.9, 0.6, -7.41170883e-01, -2.67996453e-04, 1.97769841e-05,  6.71316564e-01]>, color:[0,1,0], friction:10.0 }

juice_1 { shape:ssBox, size:[0.12, 0.12, 0.23, 0.02],,contact,mass:100.0 X:<[4.3, 3.3, 0.6, -7.41170883e-01, -2.67996453e-04, 1.97769841e-05,  6.71316564e-01]>, color:[1,1,0], friction:10.0}

juice_2 { shape:ssBox, size:[0.12, 0.12, 0.23, 0.02],,contact,mass:100.0 X:<[4.3, 3.6, 0.6, -7.41170883e-01, -2.67996453e-04, 1.97769841e-05,  6.71316564e-01]>, color:[1,1,0], friction:10.0}

juice_3 { shape:ssBox, size:[0.12, 0.12, 0.23, 0.02],,contact,mass:100.0 X:<[4.3, 3.9, 0.6, -7.41170883e-01, -2.67996453e-04, 1.97769841e-05,  6.71316564e-01]>, color:[1,1,0], friction:10.0}

#cola_1 {  shape:cylinder, size:[0.22, 0.07],, mass:100.0 X:<[1.6, 3.3, 0.6, -7.41170883e-01, -2.67996453e-04, 1.97769841e-05,  6.71316564e-01]>, color:[0,0,0], friction:10.0 }

#cola_2 {  shape:cylinder, size:[0.22, 0.07],, mass:100.0 X:<[1.6, 3.6, 0.6, -7.41170883e-01, -2.67996453e-04, 1.97769841e-05,  6.71316564e-01]>, color:[0,0,0], friction:10.0 }

#cola_3 {  shape:cylinder, size:[0.22, 0.07],, mass:100.0 X:<[1.6, 3.9, 0.6, -7.41170883e-01, -2.67996453e-04, 1.97769841e-05,  6.71316564e-01]>, color:[0,0,0], friction:10.0 }


dyna_coffee_1 {
    shape:mesh,
    X:<t(3.9 3.3 0.63) d(90 1 0 0)>,,   #absolute pose: translation by (0,0,1)
    color:[1 0 0],
    mesh:'mug.dae',
    contact,
    meshscale: 2.2,
    friction:10.0
}

dyna_coffee_2 {
    shape:mesh,
    X:<t(3.9 3.6 0.63) d(90 1 0 0)>,,   #absolute pose: translation by (0,0,1)
    color:[1 0 0],
    mesh:'mug.dae',
    contact,
    meshscale: 2.2,
    friction:10.0
}

dyna_coffee_3 {
    shape:mesh,
    X:<t(3.9 3.9 0.63) d(90 1 0 0)>,,   #absolute pose: translation by (0,0,1)
    color:[1 0 0],
    mesh:'mug.dae',
    contact,
    meshscale: 2.2,
    friction:10.0
}


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

marker_1 { shape:box, size:[0.15,0.15,0.01],,color:[0,0,1], mass:1.0, X:<t(-1.2 -0.95    0.52)>}
marker_2 { shape:box, size:[0.15,0.15,0.01],,color:[1,1,0], mass:1.0, X:<t(-1.  -1.0 0.52)>}
marker_3 { shape:box, size:[0.15,0.15,0.01],,color:[0,0,1], mass:1.0, X:<t(-1.2  0.77  0.52)>}
marker_4 { shape:box, size:[0.15,0.15,0.01],,color:[1,1,0], mass:1.0, X:<t(-1.  0.72 0.52)>}

#cola_6 { shape:ssBox, size:[0.12, 0.12, 0.23, 0.02],,mass:100.0 X:<[-1.2, -1, 0.64, -7.41170883e-01, -2.67996453e-04, 1.97769841e-05,  6.71316564e-01]>, color:[1,1,1], friction:10.0}
#box { shape:box, size:[0.1,0.1,0.1],,color:[1,1,0], mass:1.0 X:<[-0.18, -1.8, 0, 1, 0, 0, 0]>}
