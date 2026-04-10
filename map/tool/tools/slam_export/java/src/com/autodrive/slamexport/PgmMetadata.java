package com.autodrive.slamexport;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class PgmMetadata {
    private final String pgmText;
    private final int width;
    private final int height;
    private final List<Double> origin;
    private final int occupiedCells;
    private final Map<String, Double> bounds;

    public PgmMetadata(String pgmText, int width, int height, List<Double> origin, int occupiedCells, Map<String, Double> bounds) {
        this.pgmText = pgmText;
        this.width = width;
        this.height = height;
        this.origin = List.copyOf(origin);
        this.occupiedCells = occupiedCells;
        this.bounds = Collections.unmodifiableMap(new LinkedHashMap<String, Double>(bounds));
    }

    public String getPgmText() {
        return pgmText;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public List<Double> getOrigin() {
        return origin;
    }

    public int getOccupiedCells() {
        return occupiedCells;
    }

    public Map<String, Double> getBounds() {
        return bounds;
    }
}
