use bevy::prelude::*;
use crate::plugins::setup::{Particle,velocitydisplay,energydisplay};
use crate::solverdir::solver::Solver;

pub struct Updates;
impl Plugin for Updates{
    fn build(&self, app: &mut App) {
        app.add_systems(Update, upd);
    }
}
fn upd(time:Res<Time>, mut solver: ResMut<Solver>, mut query:Query<(&mut Transform, &Particle)>, mut display: Single<&mut Text, With<velocitydisplay>>,
       mut display2:Single<&mut Text, Without<velocitydisplay>>){
    println!("YO");
    solver.spawner();
    println!("Hello");
    solver.update(time.delta_secs());
    let mut str="Velocity: \n".to_string();
    let mut energy=0.0;

    for (mut transform, particle) in query.iter_mut(){
        println!("im working");
        let p=&solver.rk4particles[particle.index ];
        //updating positions
        transform.translation.x = p.pos.x;
        transform.translation.y = p.pos.y;
        //updating text screen for each ball
        energy+=solver.total_energy();
        str=str +&solver.rk4particles[particle.index].vel_tostr();
    }

    //display.0=format!("{}",str);
    //display2.0=format!("Total Energy: \n {}",energy);
}
