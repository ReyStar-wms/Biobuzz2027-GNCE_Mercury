package org.firstinspires.ftc.teamcode.config.core.util.hardware;

import java.util.*;

public final class ModeSmoother<T> {
    private final int n;
    private final Deque<T> q;
    private final Map<T, Integer> cnt;
    private final T defaultValue;

    private T mode;
    private int modeCount;

    public ModeSmoother(int windowSize, T defaultValue) {
        if (windowSize <= 0) throw new IllegalArgumentException("windowSize must be > 0");
        this.n = windowSize;
        this.defaultValue = defaultValue;
        this.q = new ArrayDeque<>(windowSize);
        this.cnt = new HashMap<>();
        this.mode = null;
        this.modeCount = 0;
    }

    public int size() {
        return q.size();
    }

    public void clear() {
        q.clear();
        cnt.clear();
        mode = null;
        modeCount = 0;
    }

    public void add(T x) {
        q.addLast(x);
        int c = cnt.merge(x, 1, Integer::sum);

        if (mode == null || c > modeCount || c == modeCount) {
            mode = x;
            modeCount = c;
        }

        if (q.size() > n) {
            T old = q.removeFirst();
            decrement(old);

            if (Objects.equals(old, mode)) {
                recomputeModeMostRecentTie();
            }
        }
    }

    public T mode() {
        return mode != null ? mode : defaultValue;
    }

    private void decrement(T x) {
        Integer c = cnt.get(x);
        if (c == null) return;
        if (c == 1) cnt.remove(x);
        else cnt.put(x, c - 1);
    }

    private void recomputeModeMostRecentTie() {
        if (q.isEmpty()) {
            mode = null;
            modeCount = 0;
            return;
        }

        int bestCount = 0;
        for (int c : cnt.values()) bestCount = Math.max(bestCount, c);

        T best = null;
        for (Iterator<T> it = q.descendingIterator(); it.hasNext();) {
            T v = it.next();
            Integer c = cnt.get(v);
            if (c != null && c == bestCount) {
                best = v;
                break;
            }
        }

        mode = best;
        modeCount = bestCount;
    }
}

