package com.autodrive.slamexport;

import java.util.LinkedHashMap;
import java.util.Map;

public final class LoadedSlam {
    private final Map<String, Object> manifest;
    private final Map<String, Object> occupancyGrid;
    private final byte[] pcdContent;

    public LoadedSlam(Map<String, Object> manifest, Map<String, Object> occupancyGrid) {
        this(manifest, occupancyGrid, null);
    }

    public LoadedSlam(Map<String, Object> manifest, Map<String, Object> occupancyGrid, byte[] pcdContent) {
        this.manifest = new LinkedHashMap<String, Object>(manifest);
        this.occupancyGrid = new LinkedHashMap<String, Object>(occupancyGrid);
        this.pcdContent = pcdContent == null ? null : pcdContent.clone();
    }

    public Map<String, Object> getManifest() {
        return manifest;
    }

    public Map<String, Object> getOccupancyGrid() {
        return occupancyGrid;
    }

    public byte[] getPcdContent() {
        return pcdContent == null ? null : pcdContent.clone();
    }
}
