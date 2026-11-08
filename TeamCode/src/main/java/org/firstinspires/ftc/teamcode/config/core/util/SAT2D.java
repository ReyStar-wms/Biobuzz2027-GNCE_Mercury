package org.firstinspires.ftc.teamcode.config.core.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Generic 2D SAT intersection for CONVEX polygons + a PolygonSet that stores STATIC polygons (axes cached).
 * - Convex polygons only (triangles, quads, etc.)
 * - Vertices must be in CW or CCW order
 * - Touching counts as intersection
 */
public final class SAT2D {

    private SAT2D() {}

    // -------------------- Geometry types --------------------

    public static final class Point2 {
        public final double x, y;
        public Point2(double x, double y) { this.x = x; this.y = y; }
    }

    /** Convex polygon with cached SAT axes (edge normals). */
    public static final class ConvexPolygon {
        private final Point2[] v;
        private final double[] ax, ay; // axis vectors (edge normals), not normalized

        public ConvexPolygon(Point2... vertices) {
            if (vertices == null || vertices.length < 3) {
                throw new IllegalArgumentException("ConvexPolygon needs >= 3 vertices.");
            }
            this.v = Arrays.copyOf(vertices, vertices.length);
            int n = v.length;
            this.ax = new double[n];
            this.ay = new double[n];
            rebuildAxes();
        }

        public Point2[] vertices() { return v; }
        public double[] axesX() { return ax; }
        public double[] axesY() { return ay; }

        private void rebuildAxes() {
            for (int i = 0; i < v.length; i++) {
                Point2 p1 = v[i];
                Point2 p2 = v[(i + 1) % v.length];
                double ex = p2.x - p1.x;
                double ey = p2.y - p1.y;
                ax[i] = -ey;
                ay[i] =  ex;
            }
        }
    }

    // -------------------- SAT intersection --------------------

    /** Returns true if polygons overlap OR touch (touching counts as intersection). */
    public static boolean intersects(ConvexPolygon a, ConvexPolygon b) {
        return !hasSeparatingAxis(a, b) && !hasSeparatingAxis(b, a);
    }

    private static boolean hasSeparatingAxis(ConvexPolygon a, ConvexPolygon b) {
        Point2[] va = a.vertices();
        Point2[] vb = b.vertices();
        double[] ax = a.axesX();
        double[] ay = a.axesY();

        for (int i = 0; i < ax.length; i++) {
            double axisX = ax[i], axisY = ay[i];

            double minA = dot(va[0], axisX, axisY), maxA = minA;
            for (int k = 1; k < va.length; k++) {
                double p = dot(va[k], axisX, axisY);
                if (p < minA) minA = p;
                if (p > maxA) maxA = p;
            }

            double minB = dot(vb[0], axisX, axisY), maxB = minB;
            for (int k = 1; k < vb.length; k++) {
                double p = dot(vb[k], axisX, axisY);
                if (p < minB) minB = p;
                if (p > maxB) maxB = p;
            }

            if (maxA < minB || maxB < minA) return true; // separated
        }
        return false;
    }

    private static double dot(Point2 p, double ax, double ay) {
        return p.x * ax + p.y * ay;
    }

    // -------------------- PolygonSet (static cached polygons) --------------------

    public static final class PolygonSet {
        private final ArrayList<Entry> entries = new ArrayList<>();

        private static final class Entry {
            final String name;
            final ConvexPolygon poly; // axes cached inside ConvexPolygon
            Entry(String name, ConvexPolygon poly) { this.name = name; this.poly = poly; }
        }

        public PolygonSet add(String name, Point2... vertices) {
            entries.add(new Entry(name, new ConvexPolygon(vertices)));
            return this;
        }

        public PolygonSet add(String name, ConvexPolygon poly) {
            entries.add(new Entry(name, poly));
            return this;
        }

        /** True if query intersects ANY polygon in this set. */
        public boolean intersectsAny(ConvexPolygon query) {
            for (Entry e : entries) {
                if (SAT2D.intersects(query, e.poly)) return true;
            }
            return false;
        }

        /** First name hit, or null if none. */
        public String firstHitName(ConvexPolygon query) {
            for (Entry e : entries) {
                if (SAT2D.intersects(query, e.poly)) return e.name;
            }
            return null;
        }

        /** All names hit (debug/telemetry). */
        public List<String> allHitNames(ConvexPolygon query) {
            ArrayList<String> hits = new ArrayList<>();
            for (Entry e : entries) {
                if (SAT2D.intersects(query, e.poly)) hits.add(e.name);
            }
            return hits;
        }

        public int size() { return entries.size(); }
        public void clear() { entries.clear(); }
    }
}
