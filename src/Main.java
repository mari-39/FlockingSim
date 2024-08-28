import processing.core.PApplet;
import java.util.ArrayList;

public class Main extends PApplet {
    ArrayList<Boid> boidList = new ArrayList<>();

    public void makeBoidList(int size) {
        for (int i = 0; i < size; i++) {
            Vector posn = new Vector(random(20, 780), random(20, 580));
            Vector vel = new Vector(random(-2, 2), random(-2, 2)); // Velocity initialization
            Boid temp = new Boid(posn, vel);
            this.boidList.add(temp);
        }
    }

    public void settings() {
        size(800, 600); // Set the window size
    }

    public void setup() {
        background(255); // Set the background color to white
        makeBoidList(100); // set up boids
    }

    public void draw() {
        background(255);
        ArrayList<Vector> toRenderPosn = Rules.move(boidList);
//        System.out.println(boidList.get(5).posn.x);
//        System.out.println(boidList.get(5).posn.y);

        for (int i = 0; i < toRenderPosn.size(); i++) {
            fill(0);
            ellipse((float) toRenderPosn.get(i).x,
                    (float) toRenderPosn.get(i).y, 5, 15);
        }
    }

    public static void main(String[] args) {
        PApplet.main("Main");
    }
}
