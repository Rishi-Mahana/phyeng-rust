mod particles;
mod plugins;
mod solverdir;
use solverdir::solver::{Solver,Constraint};
use plugins::update::Updates;
use plugins::setup::Setup;
use bevy::prelude::*;
use nalgebra::Vector2;



fn main() {
    let mut solver=Solver{ grav:Vector2::new(0.0,-250.0), rk4particles: Vec::new(),constraint:Constraint::Circle(300.0), //<==== GRAVITATIONAL CHANGES (180 is OPTIMUM EMPIRICALLY (ami maths chatu dadu))
        particles: Vec::new()
    };
    



    let mut app=App::new();
    app.add_plugins(DefaultPlugins);
    app.add_plugins(Setup);
    app.add_plugins(Updates);
    app.insert_resource(solver);
    app.run();


}






