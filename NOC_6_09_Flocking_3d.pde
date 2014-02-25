import gifAnimation.*;
Flock flock;

ArrayList<Gif> birds;


void setup() {

  size(700, 700, P3D);
  hint(ENABLE_DEPTH_SORT);
//  blendMode(SCREEN);

  birds = new ArrayList<Gif>();
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < 260; i++) {
    birds.add(new Gif(this, "bird2.gif"));
    Boid b = new Boid(width/2, height/2, 0, birds.get(i));
    flock.addBoid(b);
  }  
}

void draw() {
  background(255);
  flock.run();
  fill(0);
}

void mousePressed(){
 birds.add(new Gif(this, "bird2.gif"));
 flock.addBoid(new Boid(mouseX,mouseY,0,birds.get(birds.size()-1)));
 
}
