package com.autodrive.slamexport;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class LoadedSlam {
    private final Map<String, Object> manifest;
    private final List<SlamPoint> radarPoints;

    public LoadedSlam(Map<String, Object> manifest, List<SlamPoint> radarPoints) {
        this.manifest = new LinkedHashMap<String, Object>(manifest);
        this.radarPoints = List.copyOf(radarPoints);
    }

    public Map<String, Object> getManifest() {
        return manifest;
    }

    public List<SlamPoint> getRadarPoints() {
        return radarPoints;
    }
}
