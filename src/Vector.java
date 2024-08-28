import java.util.ArrayList;
import static java.lang.Math.sqrt;

public class Vector {

    double x;
    double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector getAverage(ArrayList<Vector> posnList) {
        double sumX = 0;
        double sumY = 0;
        for (int i = 0; i < posnList.size(); i++) {
            sumX += posnList.get(i).x;
            sumY += posnList.get(i).y;
        }
        sumX = sumX / posnList.size();
        sumY = sumY / posnList.size();

        return new Vector(sumX, sumY);
    }

// Grundrechenarten:
    public static Vector subtractVec(Vector a, Vector b) {
        return new Vector (a.x - b.x, a.y - b.y);
    }
    public static Vector addVec(Vector a, Vector b) {
        return new Vector (a.x + b.x, a.y + b.y);
    }
    public static Vector scaleVec(Vector a, double b) {
        return new Vector (a.x * b, a.y * b);
    }

// Absolutbetrag und Normalisieren:
    public static double getAbs(Vector a) {
        return sqrt(Math.pow(a.x, 2) + Math.pow(a.y, 2));
    }
    public static Vector normalizeVec(Vector a) {
        double d = getAbs(a);
        if (d == 0) {
            return new Vector(0, 0);
        }
        return scaleVec(a, 1 / d);
    }

    // checks the distances from all local boids,
    // adding the critical ones' posns to a list
    public static ArrayList<Vector> getCrit(Vector a, ArrayList<Vector> posnList) {
        ArrayList<Vector> crit = new ArrayList<>();

        for (int i = 0; i < posnList.size(); i++) {
            Vector b = subtractVec(a, posnList.get(i));
            double abs = getAbs(b);

            if (abs <= 10) {
                crit.add(posnList.get(i));
//                crit.add(b);
            }
        }
        return crit;
    }

    }