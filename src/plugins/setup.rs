use bevy::asset::AssetContainer;
use bevy::prelude::*;
use crate::solverdir::solver::{Constraint, Solver};

pub struct Setup;
impl Plugin for Setup {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, init_setup);
    }
}
#[derive(Component)]
#[derive(Debug)]
pub struct Particle{
    pub index:usize,
}
#[derive(Component)]
pub struct velocitydisplay;
#[derive(Component)]
pub struct energydisplay;
fn init_setup(mut commands:Commands, mut materials:ResMut<Assets<ColorMaterial>>, mut meshes:ResMut<Assets<Mesh>>, solver: Res<Solver>, ) {
    commands.spawn(Camera2d);

    let color=Color::hsl(0.0,0.0,0.0);
    //boundary circle
    match solver.constraint{
        Constraint::Circle(c)=>{
            commands.spawn((
                Mesh2d(meshes.add(Circle::new(c))),
                MeshMaterial2d(materials.add(Color::hsl(360.0,0.0,0.5))),
                Transform::from_xyz(0.0,0.0,0.0),
            ));
        }
        Constraint::Rect(x,y)=>{
            commands.spawn((
                Mesh2d(meshes.add(Rectangle::new(x,y))),
                MeshMaterial2d(materials.add(Color::hsl(360.0,0.0,0.5))),
                Transform::from_xyz(0.0,0.0,0.0),
            ));
        }
    }

    //all particles
    for i  in 0..solver.rk4particles.len(){
        let shapes=meshes.add(Circle::new(solver.rk4particles[i].rad));
        commands.spawn((
            Mesh2d(shapes.clone()),
            MeshMaterial2d(materials.add(color)),
            Transform::from_xyz(solver.rk4particles[i].pos.x.clone(), solver.rk4particles[i].pos.y.clone(), 0.0),

        ))
            .insert(Particle{index:i }); //tagging each particle
    }
    //velocity text screen
    commands.spawn((
        Text::new("Velocity"),
        Node{
            position_type:PositionType::Absolute,
            top:Val::Px(12.0),
            left:Val::Px(12.0),
            ..default()
        }
    )).insert(velocitydisplay);
    commands.spawn((
        Text::new("Total Energy"),
        Node{
            position_type:PositionType::Absolute,
            top:Val::Px(300.0),
            left:Val::Px(1000.0),
            ..default()
        },
    )).insert(energydisplay);
}
fn
