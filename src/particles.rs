use nalgebra::base::Vector2;
#[derive(Clone)]
pub struct Verlet{
    pub pos : Vector2<f32>,
    pub prev_pos : Vector2<f32>,
    pub acc : Vector2<f32>,
    pub rad:f32,
}

impl Verlet{
    //update position for each timestep
    pub fn update_pos(&mut self, dt:f32){
        //save pos
        let temp_pos=self.pos;
        //update
        self.pos=self.pos*2.0-self.prev_pos+self.acc*dt*dt;
        //update
        self.prev_pos=temp_pos;
        //reset acceleration, allows for updating accel at each timestep
        self.acc = Vector2::new(0.0,0.0);



    }
    //allows change in accel
    pub fn accelerate(&mut self, a:Vector2<f32>){
        self.acc=a;
    }
    pub fn velocity(&self) -> Vector2<f32>{
        self.pos-self.prev_pos
    }
    pub fn vel_tostr(&self)->String{

        format!("X: {}  Y: {} \n",self.velocity()[0], self.velocity()[1])
    }

}
pub struct RK4{
    pub pos: Vector2<f32>,
    pub vel: Vector2<f32>,
    pub acc: Vector2<f32>,
    pub rad:f32,
}
impl RK4{
    pub fn update_pos(&mut self, dt:f32){
        let k1=self.vel;
        let k2=self.vel+self.acc*dt*0.5;
        //k3=k2
        let k4=self.vel+self.acc*dt;
        self.pos+=(k1+4.0*k2+k4)*dt/6.0;
        self.vel=k4;
    }
    pub fn accelerate(&mut self, a:Vector2<f32>){
        self.acc=a;
    }
    pub fn vel_tostr(&self)->String{

        format!("X: {}  Y: {} \n",self.vel[0], self.vel[1])
    }
}