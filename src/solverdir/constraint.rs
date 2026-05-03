use nalgebra::Vector2;
use crate::solverdir::solver::Solver;
impl Solver{
    pub fn constraint_circle(&mut self, radius:f32){
        let epsilon = 0.01;
        let damp=1.0;
        for particle in self.particles.iter_mut(){
            let dist=particle.pos.norm();
            if dist>radius-particle.rad{
                let normal=particle.pos/dist;
                particle.pos = normal * (radius - particle.rad - epsilon);
                let velocity = particle.pos - particle.prev_pos;
                let v_radial=normal*velocity.dot(&normal);
                //tangential component=velocity-radial
                //new radial=-alpha  * radial
                //new velocity=(- alpha * radial) +(vel -radial)=v-(1+alpha) r
                //pos-prevpos=2r-v ===> prevpos=(1+a)r-v+ pos
                particle.prev_pos=(1.0+damp)*v_radial+particle.pos-velocity;
            }
        }
    }

    pub fn constraint_rect(&mut self, x:f32, y:f32){
        let epsilon=0.01;
        let damp=1.0;
        for particle in self.particles.iter_mut(){
            if particle.pos[0].abs()>x/2.0-particle.rad{
                let mut vel =particle.pos-particle.prev_pos;
                let unit = Vector2::new(-particle.pos[0], 0.0).normalize();
                let offset=particle.pos[0].abs()+particle.rad-x/2.0;
                particle.pos=particle.pos+unit*(offset+epsilon);

                vel[0]=-damp*vel[0];
                particle.prev_pos = particle.pos-vel;
            }
            if particle.pos[1].abs()>y/2.0-particle.rad{
                let mut vel = particle.pos -particle.prev_pos;
                let unit = Vector2::new(0.0,-particle.pos[1]).normalize();
                let offset=particle.pos[1].abs()+particle.rad-y/2.0;
                particle.pos=particle.pos+unit*(offset+epsilon);

                vel[1]=-damp*vel[1];
                particle.prev_pos = particle.pos-vel;
            }
        }
    }
    pub fn constraint_rectrk4(&mut self, x:f32, y:f32){
        let epsilon=0.0;
        let damp=0.99;
        for particle in self.rk4particles.iter_mut(){
            if particle.pos[0].abs()>x/2.0-particle.rad{
                let unit = Vector2::new(-particle.pos[0], 0.0).normalize();
                let offset=particle.pos[0].abs()+particle.rad-x/2.0;
                particle.pos=particle.pos+unit*(offset+epsilon);

                particle.vel[0]=-damp*particle.vel[0];
            }
            if particle.pos[1].abs()>y/2.0-particle.rad{
                let unit = Vector2::new(0.0,-particle.pos[1]).normalize();
                let offset=particle.pos[1].abs()+particle.rad-y/2.0;
                particle.pos=particle.pos+unit*(offset+epsilon);

                particle.vel[1]=-damp*particle.vel[1];
            }
        }
    }
    pub fn constraint_circlerk4(&mut self, radius:f32){
        let epsilon = 0.001;
        let damp=0.99;
        for particle in self.rk4particles.iter_mut(){
            let dist=particle.pos.norm();
            let normal=particle.pos/dist;
            if dist>radius-particle.rad && particle.vel.dot(&normal) >0.0{
                particle.pos = normal * (radius - particle.rad - epsilon);
                let v_radial=normal*particle.vel.dot(&normal);
                //tangential component=velocity-radial
                //new radial=-alpha  * radial
                //new velocity=(- alpha * radial) +(vel -radial)=v-(1+alpha) r
                particle.vel=particle.vel - (1.0+damp)*v_radial;
            }
        }
    }
}