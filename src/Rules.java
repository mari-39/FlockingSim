import java.util.ArrayList;

public class Rules {
    static int NEIGHBOR_RAD = 80;
    static double SPEED_LIMIT = 2.0;
    static int WIDTH = 800;
    static int BREADTH = 600;
    static Vector CENTER = new Vector(WIDTH / 2, BREADTH / 2);

    // 1. separation
    // for boids in a NEIGHBOR-pixel rad, move away in the opposite direction
// returns: normalized direction-vec to avoid collisions
    public static Vector separationConstraint(Vector a, ArrayList<Vector> posnList) {
        if (posnList.isEmpty()) {
            return new Vector(0, 0);
        }

        double totalWeight = 0;
        Vector scaledMovement = new Vector(0, 0);

        for (int i = 0; i < posnList.size(); i++) {
            double weight;
            Vector between = Vector.subtractVec(a, posnList.get(i));
            double dist = Vector.getAbs(between);
            double epsilon = 1e-6;
            if (dist < epsilon) {
                weight = 1 / epsilon;
            }
            else {
                weight = 1 / dist;
            }
            Vector weightedVec = Vector.scaleVec(between, weight);

            // aggregate all vecs by adding them up
            scaledMovement = Vector.addVec(scaledMovement, weightedVec);
            totalWeight += weight;
        }
        Vector nonNormalTemp = Vector.scaleVec(scaledMovement, 1 / totalWeight);
        return Vector.normalizeVec(nonNormalTemp);
    }

    // 2. cohesion
    // steer towards average posn of boids in 10-p. rad
    // needs a starting point => not the same as alignment
    public static Vector cohesionConstraint(ArrayList<Vector> posnList, Boid b) {
        Vector average = Vector.getAverage(posnList);
        return Vector.normalizeVec(Vector.subtractVec(average, b.posn));
    }

    // 3. alignment
    // make vel the average heading / vel of local boids
    public static Vector alignmentConstraint(ArrayList<Vector> velList) {
        Vector average = Vector.getAverage(velList);
        return Vector.normalizeVec(average);
    }

    // 4. central tendency
    // lightly push the boid toward the center, weighing the force by distance
    public static Vector centralForce(Boid b) {
        Vector subVec = Vector.subtractVec(CENTER, b.posn);
        double dist = Vector.getAbs(subVec);
        double strength = 0.004;
        return Vector.scaleVec(Vector.normalizeVec(subVec), dist * strength);
    }


    // filter functions for neighbors
    public static ArrayList<Vector> filterPosn(Vector a, ArrayList<Boid> b) {
        ArrayList<Vector> posnList = new ArrayList<>();

        for (int i = 0; i < b.size(); i++) {
            Vector tempPosn = b.get(i).posn;
            Vector sub = Vector.subtractVec(a, tempPosn);
            double dist = Vector.getAbs(sub);

            if (dist < NEIGHBOR_RAD) {
                posnList.add(tempPosn);
            }
        }
        return posnList;
    }


    public static ArrayList<Vector> filterVel(Vector a, ArrayList<Boid> b) {
        ArrayList<Vector> velList = new ArrayList<>();

        for (int i = 0; i < b.size(); i++) {
            Vector tempVel = b.get(i).vel;
            Vector tempPosn = b.get(i).posn;
            Vector sub = Vector.subtractVec(a, tempPosn);
            double dist = Vector.getAbs(sub);

            if (dist < NEIGHBOR_RAD) {
                velList.add(tempVel);
            }
        }
        return velList;
    }

    public static Vector speedLimit(Vector a) {
        // get SPEED (movement / time)
        double velMagnitude = Vector.getAbs(a);
        if (velMagnitude > SPEED_LIMIT) {
            // scale the normalized vec to conserve direction
            return Vector.scaleVec(Vector.normalizeVec(a), SPEED_LIMIT);
        }
        return a;
    }

    public static Vector borderPatrol(Boid b) {
        if (b.posn.x < 20 || b.posn.x > 780) {
            b.vel.x = - b.vel.x;
        }
        if (b.posn.y < 20 || b.posn.y > 580) {
            b.vel.y = - b.vel.y;
        }
        return b.vel;
    }

    public static ArrayList<Vector> move(ArrayList<Boid> boids) {
        ArrayList<Vector> toRenderPosn = new ArrayList<>();
        for (int i = 0; i < boids.size(); i++) {
            Boid current = boids.get(i);
            ArrayList<Boid> boids2 = new ArrayList<>(boids);
            boids2.remove(i);
            // ArrayList of all neighbors' positions
            ArrayList<Vector> neighborPosn = filterPosn(current.posn, boids2);
            // AL of all n. vel
            ArrayList<Vector> neighborVel = filterVel(current.posn, boids2);

            // 1. separationConstraint using neighborPosn
            Vector separationForce = separationConstraint(current.posn, neighborPosn);
            // !!!!!!!!!!!!!
            // X DOUBT:
            separationForce = Vector.scaleVec(separationForce, 1.1);

            // 2. cohesionConstraint using posn
            Vector cohesionForce = cohesionConstraint(neighborPosn, current);
            cohesionForce = Vector.scaleVec(cohesionForce, 0.5);

            // 3. alignmentConstraint using vel
            Vector alignmentForce = alignmentConstraint(neighborVel);
            alignmentForce = Vector.scaleVec(alignmentForce, 0.5);


            // 4. centralConstraint
            Vector centrumForce = centralForce(current);

            Vector totalAcc = Vector.addVec(centrumForce, Vector.addVec(Vector.addVec(separationForce, alignmentForce), cohesionForce));
            // for controlling acceleration scale by 0.1:
            totalAcc = Vector.scaleVec(totalAcc, 0.1);

            current.vel = Vector.addVec(current.vel, totalAcc);
            current.vel = speedLimit(current.vel);
            current.posn = Vector.addVec(current.posn, current.vel);
            current.vel = borderPatrol(current);

            toRenderPosn.add(current.posn);
        }
        return toRenderPosn;
    }
}
