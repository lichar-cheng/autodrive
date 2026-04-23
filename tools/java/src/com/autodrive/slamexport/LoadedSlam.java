package com.autodrive.slamexport;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class LoadedSlam {
    private final Map<String, Object> manifest;
    private final Map<String, Object> occupancyGrid;

    public LoadedSlam(Map<String, Object> manifest, Map<String, Object> occupancyGrid) {
        this.manifest = new LinkedHashMap<String, Object>(manifest);
        this.occupancyGrid = new LinkedHashMap<String, Object>(occupancyGrid);
    }

    public Map<String, Object> getManifest() {
        return manifest;
    }

    public Map<String, Object> getOccupancyGrid() {
        return occupancyGrid;
    }
}
