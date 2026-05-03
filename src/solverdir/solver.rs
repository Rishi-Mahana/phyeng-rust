use nalgebra::Vector2;
use crate::particles::{Verlet, RK4};
use bevy::prelude::Resource;

pub enum Constraint{
    Circle(f32),
    Rect(f32,f32),
}
#[derive(Resource)]
pub struct Solver{
    //THIS IS WHERE GRAVITATIONAL CONSTANT IS SET
    pub grav:Vector2<f32>,
    pub rk4particles:Vec<RK4>,
    pub constraint:Constraint,
    pub particles:Vec<Verlet>,
}
impl Solver{
    //add function basically

    pub fn add_rk4(&mut self, pos: Vector2<f32>, vel: Vector2<f32>, rad:f32){
        let particle=RK4{
            pos:pos,
            vel:vel,
            acc:Vector2::new(0.0,0.0),
            rad:rad,
        };
        self.rk4particles.push(particle);
    }
    //combined update
    pub fn update(&mut self, dt:f32){
        self.apply_grav();
        self.update_pos(dt);
        match self.constraint{
            Constraint::Circle(c)=>{self.constraint_circlerk4(c)},
            Constraint::Rect(x,y)=>{self.constraint_rectrk4(x,y)}
        }
        
        self.solve_collision_naive_rk4();
    }
    pub fn update_pos(&mut self, dt:f32){
        self.rk4particles.iter_mut().for_each(|rk4| rk4.update_pos(dt));
    }
    pub fn apply_grav(&mut self){
        self.rk4particles.iter_mut().for_each(|rk4| rk4.accelerate(self.grav));
    }
    pub fn total_energy(&self)-> f32{
        let ground= match(&self.constraint){
            Constraint::Circle(x)=>{*x},
            Constraint::Rect(x,y)=>{y*0.5},

        };
        let mut energy=0.0;
        for particle in &self.rk4particles{
            energy+=(self.grav.magnitude()*(particle.pos[1]+ground))+
             (0.5*(particle.vel.magnitude_squared()));
        }
        energy

    }
    pub fn spawner(&mut self){
        if (self.rk4particles.len()<2) {
            let particle = RK4 {
                pos: Vector2::new(0.0, 0.0),
                vel: Vector2::new(0.0, 0.0),
                acc:Vector2::new(0.0, 0.0),
                rad: 1.0,
            };
            self.rk4particles.push(particle);
        }
    }



    


    pub fn solve_collision_naive_verlet(&mut self){
        for i in  0..self.particles.len(){
            for j in i+1..self.particles.len(){
                let (p1,p2)=(&self.particles[i],&self.particles[j]);
                let p1to2=p2.pos-p1.pos;
                let dist=p1to2.norm();
                let damp=0.999;
                if dist<p1.rad+p2.rad{
                    let unit12:Vector2<f32>= if dist!=0.0{
                        p1to2/dist
                    }else{
                        Vector2::new(1.0,0.0) //if the objects are exactly overlapping then arbitrary unit vector
                    };
                    let scale=(p1.rad+p2.rad-dist)/2.0;
                    let vel1=self.particles[i].pos - self.particles[i].prev_pos;
                    let vel2=self.particles[j].pos - self.particles[j].prev_pos;
                    let v1_unit12=unit12*vel1.dot(&unit12);
                    let v2_unit21=-unit12*vel2.dot(&-unit12);
                    //remaining vel of 1: vel1-along12 ===> new velalong12 = -a * v12 ===> new velocity = (vel1-along12)+(new ) = vel1- v12 -  a * v12  ===> prev = pos +(1+a)v -v1
                    //remaining vel of 2: vel2- along21 ===> new velalong21 = - a * v21 ===> new velocity =(vel2-along21) +(new ) = vel2 - v21 - a * v21 ===> prev= = pos +(1+a)v - v2
                    self.particles[i].prev_pos=self.particles[i].pos +(1.0+damp)*v1_unit12 - vel1;
                    self.particles[j].prev_pos=self.particles[j].pos +(1.0+damp)*v2_unit21 - vel2;
                    self.particles[i].pos-=unit12*scale;
                    self.particles[j].pos+=unit12*scale;



                }
            }
        }

    }
    pub fn solve_collision_naive_rk4(&mut self){
        for i in  0..self.rk4particles.len(){
            for j in i+1..self.rk4particles.len(){
                let (p1,p2)=(&self.rk4particles[i],&self.rk4particles[j]);
                let p1to2=p2.pos-p1.pos;
                let dist=p1to2.norm();
                let damp=0.99;
                if dist<p1.rad+p2.rad{
                    let unit12:Vector2<f32>= if dist!=0.0{
                        p1to2/dist
                    }else{
                        Vector2::new(1.0,0.0) //if the objects are exactly overlapping then arbitrary unit vector
                    };
                    let scale=(p1.rad+p2.rad-dist)/2.0;


                    self.rk4particles[i].pos-=unit12*scale;
                    self.rk4particles[j].pos+=unit12*scale;

                    let rel_vel = (self.rk4particles[i].vel - self.rk4particles[j].vel).dot(&unit12);  //suggested by claude
                    if rel_vel>0.0 {
                        let u1 = unit12 * self.rk4particles[i].vel.dot(&unit12);
                        let u2 = unit12 * self.rk4particles[j].vel.dot(&unit12);
                        let u1_t = self.rk4particles[i].vel - u1;
                        let u2_t = self.rk4particles[j].vel - u2;
                        let v1 = ((1.0 + damp) * u2 + (1.0 - damp) * u1) * 0.5;
                        let v2 = ((1.0 + damp) * u1 + (1.0 - damp) * u2) * 0.5;
                        self.rk4particles[i].vel = v1 + u1_t;
                        self.rk4particles[j].vel = v2 + u2_t;
                    }
                }
            }
        }

    }

}